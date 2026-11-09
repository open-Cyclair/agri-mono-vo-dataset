# Mono-Camera KITTI-like Conversion

Convert a ROS 2 MCAP bag with one camera stream into a KITTI-like sequence
with `scripts/mono_camera_kitti_converter.py`.

The converter is for bags where the only image topic is the nav1 HD camera.
Default topics:

| Role       | Topic                            | Type                       |
|------------|----------------------------------|----------------------------|
| Image      | `/cam_nav1/hd/image_color_rect`  | `sensor_msgs/msg/Image`      |
| CameraInfo | `/cam_nav1/hd/camera_info`       | `sensor_msgs/msg/CameraInfo` |
| Odometry   | `/odom/sbg`                      | `nav_msgs/msg/Odometry`      |
| TF         | `/tf_static`, `/tf`              | `tf2_msgs/msg/TFMessage`     |

Images are written to disk as the bag is read, so the converter never holds
the full image sequence in RAM.

The bag can be complete or partial (only some chunks downloaded, see
[download.md](download.md#downloading-part-of-a-sequence)). The converter
reads the chunks that are present, in order, and lists the missing ones in
`info.txt`. A single `.mcap` file can also be passed to `--bag`.

## Requirements

The simplest way is the Docker image built from this repository (see
[Running in Docker](#running-in-docker)). It contains every dependency at a
fixed version.

To run the converter without Docker, on Ubuntu 22.04 with ROS 2 Humble:

```bash
sudo apt install ros-humble-rosbag2-storage-mcap ros-humble-cv-bridge \
                 ros-humble-tf-transformations
pip install -r requirements-converter.txt
source /opt/ros/humble/setup.bash
```

`requirements-converter.txt` pins `transforms3d` (used by
`tf_transformations`) and `mcap` and `mcap-ros2-support`, which decode the SBG
and Cyclair topics in the `topics/` export.

## Usage

```bash
python3 scripts/mono_camera_kitti_converter.py \
  --bag <bag_dir> \
  --out <output_root> \
  --seq <sequence_name>
```

`<bag_dir>` is the downloaded bag folder that holds `metadata.yaml` and all
or some of the `.mcap` chunks (see [download.md](download.md)), or a single
`.mcap` file.

The defaults match the dataset bags. Override topics only if a bag uses
different names:

```bash
python3 scripts/mono_camera_kitti_converter.py \
  --bag <bag_dir> \
  --out <output_root> \
  --seq <sequence_name> \
  --image-topic /cam_nav1/hd/image_color_rect \
  --camera-info-topic /cam_nav1/hd/camera_info \
  --odom-topic /odom/sbg
```

The sequence folder must not exist before you run the converter. It refuses
to overwrite an existing folder.

## Output Layout

The output is written under:

```text
<output_root>/sequences/<sequence_name>/
```

```text
sequence_name/
├── image_/                     PNG frames (000000.png, 000001.png, ...) from the camera topic
├── calib.txt                   P2 projection matrix (3x4, row-major) from CameraInfo
├── poses.txt                   one 3x4 pose per frame (12 values, row-major)
├── times.txt                   one timestamp per image_ frame (seconds)
├── info.txt                    conversion metadata (source bag, topics, frames)
├── nav_camera_to_imu_link.txt  TF transform between the nav camera and imu_link
├── README.md                   per-sequence conversion notes
└── topics/                     optional text export of unused topics
```

There is intentionally no `image_0/`, `image_1/` or `image_2/` folder. This
dataset has one camera, and its frames go to `image_/`.

### File conventions

- **`times.txt`**: the image `header.stamp` in seconds (9 decimals). This is
  absolute ROS time, not time since the start of the sequence.
- **`calib.txt`**: one line, `P2: p00 p01 ... p23`, holding the `P` matrix of the
  first valid `CameraInfo` message. The `P2` key is kept for compatibility with
  KITTI loaders. The images come from the rectified topic, so `P` applies to
  them directly.
- **`poses.txt`**: for each image, the `/odom/sbg` pose with the nearest
  timestamp, written as the top 3x4 block of the homogeneous matrix (KITTI
  format). The pose is the odometry message's `pose.pose`, expressed in its
  `header.frame_id` for its `child_frame_id`. It is **not** moved into the
  camera frame. Use `nav_camera_to_imu_link.txt` for that.
  If no odometry message exists, the identity pose is written.
- **`nav_camera_to_imu_link.txt`**: the camera ↔ robot-centre transform, built
  from `/tf_static` and `/tf`. It gives both `T_imu_link_camera` and
  `T_camera_imu_link` as translation, quaternion (`x y z w`) and 4x4 matrix.
  Convention: `T_target_source` maps coordinates from the source frame to the
  target frame. If no TF chain is found, `status: not_found` and the list of
  available frames are written.

## Exporting Unused Topics

By default, every topic not used for KITTI images, calibration or poses is
exported as text under:

```text
<output_root>/sequences/<sequence_name>/topics/
```

Each topic gets one `.txt` file. The file contains:

- topic name
- ROS message type
- bag timestamp
- message timestamp when available
- deserialized message text
- raw message byte size if the message cannot be decoded

Messages are decoded with the definitions stored in the MCAP files, so the SBG
(`sbg_driver/msg/*`) and Cyclair messages are read without installing their
ROS packages. This requires the Python packages `mcap` and
`mcap-ros2-support`. Without them, only the message types installed in the ROS
environment are decoded. The `sbg_driver` package available for ROS 2 Humble
does not match the version used on the robot, so do not rely on it to read
these topics.

Disable this export with:

```bash
--no-export-unused-topics
```

## Nav Camera To Robot Center TF

By default the robot centre frame is `imu_link`. The camera frame is read
from the Image or CameraInfo `header.frame_id`. Override them only if needed:

```bash
--robot-frame imu_link --camera-frame <camera_frame_id>
```

## Running in Docker

The `Dockerfile` at the root of the repository builds an image with ROS 2
Humble (base image pinned by digest), the MCAP storage plugin and the Python
packages of `requirements.txt` and `requirements-converter.txt`. Build it
once, from the repository root:

```bash
docker build -t gs-agrifield-tools .
```

Then convert a bag stored in `<data_dir>`:

```bash
docker run --rm --memory=8g \
  --user "$(id -u):$(id -g)" \
  -v <data_dir>:/data \
  gs-agrifield-tools convert \
    --bag /data/<bag_name> \
    --out /data/kitti \
    --seq <bag_name>
```

`--user` writes the output files as your user. `<data_dir>` must exist
before the first run. The memory limit keeps the machine responsive.
`docker run --rm gs-agrifield-tools convert --help` lists all options.

### Linux, macOS and Windows

The commands use bash syntax. On Linux and macOS, run them in a terminal. On
Windows, run them in a WSL 2 terminal (Ubuntu), which Docker Desktop already
requires; they do not work as written in PowerShell or `cmd`. On macOS and
Windows, give Docker Desktop at least 8 GB of memory (Settings > Resources).
The base image is available for x86-64 and ARM64 (including Apple silicon).

> **Note:** `info.txt` and the per-sequence `README.md` record the absolute
> path of the source bag (`/data/...` in Docker). Check these
> files before you redistribute converted sequences.

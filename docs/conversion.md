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

## Requirements

- ROS 2 Humble, with the MCAP storage plugin
- `cv_bridge`, `tf_transformations`, OpenCV, NumPy

On Ubuntu 22.04:

```bash
sudo apt install ros-humble-rosbag2-storage-mcap ros-humble-cv-bridge \
                 ros-humble-tf-transformations
pip install transforms3d   # needed by tf_transformations
source /opt/ros/humble/setup.bash
```

## Usage

```bash
python3 scripts/mono_camera_kitti_converter.py \
  --bag <bag_dir> \
  --out <output_root> \
  --seq <sequence_name>
```

`<bag_dir>` is the downloaded bag folder that holds `metadata.yaml` and the
`.mcap` file(s) (see [download.md](download.md)).

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
- deserialized message text when the type is available
- raw message byte size when the type is not available

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

## Running In Docker

If ROS 2 is not installed on the host, run the converter in a ROS 2 Humble
container and mount the repository and data folders:

```bash
docker run --rm \
  --memory=8g --memory-swap=8g \
  -e PYTHONUNBUFFERED=1 \
  -v "$(pwd)":/workspace \
  -v <data_dir>:/data \
  -w /workspace \
  ros:humble \
  bash -lc 'apt-get update && apt-get install -y python3-pip \
              ros-humble-rosbag2-storage-mcap ros-humble-cv-bridge ros-humble-tf-transformations \
            && pip install transforms3d \
            && source /opt/ros/humble/setup.bash \
            && python3 scripts/mono_camera_kitti_converter.py \
                 --bag "/data/<bag_name>" \
                 --out "/data/<bag_name>_KITTI" \
                 --seq "<bag_name>"'
```

The memory limit keeps the machine responsive if a future change starts
buffering too much data by accident.

> **Note:** `info.txt` and the per-sequence `README.md` record the absolute
> path of the source bag on the machine that ran the conversion. Check these
> files before you redistribute converted sequences.

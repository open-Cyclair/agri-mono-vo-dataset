# GS-AgriField: Monocular Visual Odometry in Agricultural Environments

> **Status:** the paper describing the dataset is about to be submitted. Until it
> is published, please cite it as the manuscript below; the reference will be
> updated after publication.

This repository accompanies the **GS-AgriField** dataset and the paper *GS-AgriField: A dataset and benchmark for visual localization of an agricultural robot in row crops*.
It contains documentation and utilities to:

1. **download** the raw ROS 2 bags (MCAP) from the MARV Robotics server, and
2. **convert** them into a KITTI-like monocular sequence format.

The datasets themselves are **not** stored in this repository.

---

## Download Dataset

**➡️ [Download the dataset on MARV Robotics](https://marv.mesocentre.uca.fr/?filter=#/collection/cyclair)**

The data is hosted on a MARV Robotics server in Clermont-Ferrand, France.

**Access.** Access to the dataset is free. A request form will be online soon:
it will send the MARV access link and credentials automatically by email,
without manual approval.

| Seq. | Bag name | Crop | BBCH | Speed | Date |
|------|----------|------|------|-------|------|
| 01 | `vo_dataset_2026-06-08_pressac_sugarbeet_bbch19+_fast_run001` | sugar beet | 19+ | fast | 2026-06-08 |
| 02 | `vo_dataset_2026-06-08_pressac_maize_bbch14_nominal_run001` | maize | 14 | nominal | 2026-06-08 |
| 03 | `vo_dataset_2026-06-08_pressac_maize_bbch14_fast_run002` | maize | 14 | fast | 2026-06-08 |
| 04 | `vo_dataset_2026-06-08_pressac_maize_bbch14-15_fast_run003` | maize | 14-15 | fast | 2026-06-08 |
| 05 | `vo_dataset_2026-06-16_pressac_maize_bbch12_fast_run001` | maize | 12 | fast | 2026-06-16 |
| 06 | `vo_dataset_2026-06-16_pressac_maize_bbch12_nominal_run001` | maize | 12 | nominal | 2026-06-16 |
| 07 | `vo_dataset_2026-06-16_pressac_maize_bbch18_fast_run001` | maize | 18 | fast | 2026-06-16 |
| 08 | `vo_dataset_2026-06-16_pressac_sunflower_bbch10_fast_run001` | sunflower | 10 | fast | 2026-06-16 |
| 09 | `vo_dataset_2026-07-08_pressac_maize_bbch19+_fast_run001` | maize | 19+ | fast | 2026-07-08 |
| 10 | `vo_dataset_2026-07-08_pressac_maize_bbch19+_fast_run002` | maize | 19+ | fast | 2026-07-08 |
| 11 | `vo_dataset_2026-07-08_pressac_sunflower_bbch19+_fast_run001` | sunflower | 19+ | fast | 2026-07-08 |

Each sequence has its own download page in the MARV collection.

To fetch a sequence from the command line, see [docs/download.md](docs/download.md).

---

## Dataset Overview

GS-AgriField was recorded by two Cyclair GS weeding robots, four-wheel
independently steered and driven, during their normal weeding work in maize,
sunflower and sugar beet fields at Pressac (Nouvelle-Aquitaine, France), on
8 June, 16 June and 8 July 2026. The growth stages range from emergence
(BBCH 10–12) to nine or more unfolded leaves (BBCH 19+).

- **11 sequences**, 61 minutes and 1.9 km in total, about 540 GB of ROS 2 MCAP recordings.
- **Camera:** forward-looking Lucid Vision Labs Triton HDR camera (Sony IMX490),
  rectified 2880 × 1860 color images (5.4 MP) at 10 Hz, with calibration.
- **Reference trajectory:** SBG Systems Ellipse-D GNSS-aided INS with dual-antenna RTK GNSS.
- **Proprioception:** inertial measurements and wheel steering and traction feedback.

Each sequence is a ROS 2 bag (MCAP storage). The topics used by the KITTI-like
converter are:

| Role       | Topic                            | Message type                 |
|------------|----------------------------------|------------------------------|
| Camera     | `/cam_nav1/hd/image_color_rect`  | `sensor_msgs/msg/Image`      |
| Intrinsics | `/cam_nav1/hd/camera_info`       | `sensor_msgs/msg/CameraInfo` |
| Odometry   | `/odom/sbg`                      | `nav_msgs/msg/Odometry`      |
| Transforms | `/tf_static`, `/tf`              | `tf2_msgs/msg/TFMessage`     |

## Repository Structure

```text
.
├── README.md
├── LICENSE                 BSD 3-Clause (code)
├── LICENSE-DATA            CC BY-NC-SA 4.0 (dataset)
├── CITATION.cff
├── requirements.txt        Python dependencies of the downloader
├── scripts/
│   ├── download_marv.py                 download a ROS 2 bag from MARV
│   └── mono_camera_kitti_converter.py   ROS 2 bag -> KITTI-like sequence
└── docs/
    ├── download.md         downloading the ROS 2 bags
    └── conversion.md       KITTI-like conversion, file formats and conventions
```

## Quick Start

### 1. Download a ROS 2 bag

```bash
pip install -r requirements.txt
export MARV_USER=<user>
python3 scripts/download_marv.py <dataset_hash> --output-dir data/bags
```

### 2. Convert it to a KITTI-like sequence

Requires ROS 2 Humble with the MCAP storage plugin (see
[docs/conversion.md](docs/conversion.md), including a Docker option).

```bash
source /opt/ros/humble/setup.bash
python3 scripts/mono_camera_kitti_converter.py \
  --bag data/bags/<bag_name> \
  --out data/kitti \
  --seq <bag_name>
```

### KITTI-like output

```text
data/kitti/sequences/<sequence_name>/
├── image_/      PNG frames 000000.png, 000001.png, ...
├── calib.txt    P2: 3x4 projection matrix from CameraInfo
├── poses.txt    3x4 pose per frame (KITTI format), nearest /odom/sbg message
├── times.txt    timestamp per frame (s)
└── info.txt     conversion metadata
```

The single camera goes to `image_/`, not `image_2/`. The converter also writes
`nav_camera_to_imu_link.txt` (camera ↔ `imu_link` extrinsics from TF), a
per-sequence `README.md`, and an optional `topics/` text export. See
[docs/conversion.md](docs/conversion.md) for every file format and frame
convention.

## License

- **Code** (`scripts/`): [BSD 3-Clause License](LICENSE).
- **Dataset** (ROS 2 bags hosted on MARV, and the data derived from them):
  [Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
  (CC BY-NC-SA 4.0)](LICENSE-DATA).

## Citation

If you use this dataset, please cite:

```bibtex
@misc{dalah2026gsagrifield,
  title  = {{GS-AgriField}: A dataset and benchmark for visual localization of an agricultural robot in row crops},
  author = {Dalah, Aya and Alwan, Salwan and Debain, Christophe and Laneurit, Jean},
  year   = {2026},
  note   = {Manuscript in preparation},
  url    = {https://github.com/itsayoushka/agri-mono-vo-dataset}
}
```

See also [CITATION.cff](CITATION.cff).

## Affiliations and Acknowledgements

Aya Dalah<sup>1,2</sup>, Salwan Alwan<sup>2</sup>, Christophe Debain<sup>1</sup> and Jean Laneurit<sup>1</sup>

1. INRAE, Technologies and Information Systems for Agroecosystems, Clermont-Ferrand, France
2. Cyclair SAS, Poitiers, France

This work was supported by a CIFRE PhD fellowship funded by the ANRT
(Association Nationale de la Recherche et de la Technologie) and conducted in
collaboration with Cyclair SAS. It has been partially supported by ROBOTEX 2.0,
the French Infrastructure in Robotics, under the grants ROBOTEX (EQUIPEX
ANR-10-EQPX-44-01) and TIRREX (EQUIPEX+ grant ANR-21-ESRE-0015).

Data hosting: MARV Robotics server, Clermont-Ferrand, France.

## Contact

Aya Dalah, aya.dalah@cyclair.fr

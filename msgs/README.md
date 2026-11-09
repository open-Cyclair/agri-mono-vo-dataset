# Custom ROS 2 Messages

The bags contain topics whose types are not part of standard ROS 2. The images,
poses and calibration do not use them, so the KITTI-like conversion works
without these definitions. They are needed only to read the topics below.

| Topic | Type | Content |
|-------|------|---------|
| `/swerve_data_nav` | `navigation_controller_interfaces/msg/SwerveDataNav` | Steering and traction targets and feedback of the four wheels |
| `/cam_nav1/hd/exif_info` | `rovers_msgs/msg/ExifInfo` | Camera gain and exposure time, image brightness and sharpness indicators |

The `.msg` files are stored here under their original package names, because
the bags refer to these names. `SwerveDataNav` uses `MotorsCommand`, from the
same package.

## Units of `/swerve_data_nav`

The topic is published at 25 Hz. `feedbacks` holds the measured values and
`targets` the commands, for the four wheels `fl`, `fr`, `rl` and `rr`.

| Field | Unit | Sign | Resolution |
|-------|------|------|------------|
| `*_rate_trac` (wheel speed) | rad/s | positive when the wheel rolls forward | about 0.0021 rad/s |
| `*_angle_dir` (steering angle) | rad | positive counterclockwise about +z | about 0.00175 rad (0.1°) |

Speeds are given at the wheel, not at the motor. Multiply `*_rate_trac` by the
wheel radius to obtain a linear speed. The `.msg` files do not state units: the
values above were derived from the recorded data.

## SBG Messages

Topics of type `sbg_driver/msg/*` come from the public
[SBG Systems ROS 2 driver](https://github.com/SBG-Systems/sbg_ros2_driver).

## Reading These Topics

The KITTI-like converter decodes these topics, and the SBG ones, in its
`topics/` text export, using the definitions stored in the MCAP files (see
[docs/conversion.md](../docs/conversion.md#exporting-unused-topics)).

## Reading These Topics Without ROS

The Python library [rosbags](https://pypi.org/project/rosbags/) can load the
definitions directly. Run this from the repository root:

```python
from pathlib import Path

from rosbags.typesys import Stores, get_types_from_msg, get_typestore

typestore = get_typestore(Stores.ROS2_HUMBLE)
types = {}
for path in Path("msgs").glob("*/msg/*.msg"):
    types.update(get_types_from_msg(path.read_text(), f"{path.parts[-3]}/msg/{path.stem}"))
typestore.register(types)

# raw_data: the serialized (CDR) payload of one message read from the bag
msg = typestore.deserialize_cdr(raw_data, "navigation_controller_interfaces/msg/SwerveDataNav")
print(msg.feedbacks.fl_angle_dir, msg.feedbacks.fl_rate_trac)
```

To use these messages with ROS 2 tools, build a package with the same name
around the `.msg` files.

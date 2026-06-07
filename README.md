# RTK Mapping

LiDAR mapping system that uses **RTK/GNSS positioning** to build globally-aligned point-cloud maps in real time. Built on [FAST_LIO](https://github.com/hku-mars/FAST_LIO) with an added **GNSS conversion** layer that transforms raw `sensor_msgs/NavSatFix` data into zero-based odometry.

## Overview

```
GNSS NavSatFix ──→ gnss_conversion ──→ /rtk_odom ──→ FAST_LIO (RTK mode)
(lat/lon/alt)     (WGS84→ENU,         (Odometry    (LiDAR mapping with
                   zero-origin)        from 0,0,0)   absolute position)
```

- **`FAST_LIO`** — LiDAR-inertial odometry with an RTK mapping mode that replaces EKF state with external position fixes for globally-registered mapping.
- **`gnss_conversion`** — Converts raw GNSS fixes (`sensor_msgs/NavSatFix`) into local ENU odometry (`nav_msgs/Odometry`) relative to the first received fix. Handles WGS84→ECEF→ENU transformation, heading-from-motion, antenna offset, jump detection, and origin reset.

## Requirements

- **ROS Noetic** (catkin workspace)
- **Eigen3**
- **PCL ≥ 1.8**
- **Livox SDK** (for Livox LiDAR; see [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK))

## Build

```bash
# Clone into your catkin workspace
cd ~/catkin_ws/src
git clone --recursive https://github.com/<your-org>/rtk_mapping.git
cd ..

# Build
catkin_make
# or: catkin build

source devel/setup.bash
```

## Usage

### 1. Launch GNSS conversion

```bash
roslaunch gnss_conversion gnss_conversion.launch \
    input_rtk_topic:=/your_gnss_fix \
    output_odom_topic:=/rtk_odom
```

Configurable parameters (see `gnss_conversion/config/gnss_conversion.yaml`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_rtk_topic` | `/rtk_fix` | Input GNSS topic (`sensor_msgs/NavSatFix`) |
| `output_odom_topic` | `/rtk_odom` | Output odometry topic (`nav_msgs/Odometry`) |
| `publish_tf` | `true` | Broadcast `odom→base_link` TF |
| `jump_threshold` | `50.0` | Reset origin on position jump (meters) |
| `heading_min_speed` | `0.5` | Min speed (m/s) to compute heading from motion |
| `antenna_offset_x/y/z` | `0.0` | GNSS antenna offset in base_link frame |

### 2. Launch RTK mapping (FAST_LIO)

```bash
roslaunch fast_lio mapping_velodyne_rtk.launch
```

### 3. Combined launch

```bash
roslaunch gnss_conversion gnss_conversion.launch & 
roslaunch fast_lio mapping_velodyne_rtk.launch
```

## Packages

| Package | Description |
|---------|-------------|
| [`FAST_LIO`](FAST_LIO/) | LiDAR-inertial odometry + RTK mapping mode (modified from [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)) |
| [`gnss_conversion`](gnss_conversion/) | GNSS NavSatFix → zero-based odometry conversion |

## Coordinate Frames

- **`odom`** — Local ENU frame with origin at the first GNSS fix
- **`base_link`** — Vehicle body frame (TF broadcast by `gnss_conversion`)
- **`camera_init`** — FAST_LIO map frame (first LiDAR scan pose)

## License

FAST_LIO is distributed under BSD license. See [FAST_LIO/LICENSE](FAST_LIO/LICENSE).

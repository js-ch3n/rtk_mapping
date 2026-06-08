# RTK Mapping

LiDAR mapping system that uses **RTK/GNSS positioning** to build globally-aligned point-cloud maps in real time. Built on [FAST_LIO](https://github.com/hku-mars/FAST_LIO) with an added **GNSS conversion** layer that transforms raw `sensor_msgs/NavSatFix` data into zero-based odometry.

## Overview

```
GNSS NavSatFix ──→ gnss_conversion ──→ /rtk_odom ──→ FAST_LIO (RTK mode)
(lat/lon/alt)     (WGS84→ENU,         (Odometry    (LiDAR mapping with
                   zero-origin)        from 0,0,0)   absolute position)
```

- **`FAST_LIO`** — LiDAR-inertial odometry supporting **three RTK operating modes** (see below). Modified from [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO).
- **`gnss_conversion`** — Converts raw GNSS fixes (`sensor_msgs/NavSatFix`) into local ENU odometry (`nav_msgs/Odometry`) relative to the first received fix. Handles WGS84→ECEF→ENU transformation, heading-from-motion, antenna offset, jump detection, and origin reset.

> **Note:** `gnss_conversion` currently provides **3-DOF position only** (ENU x, y, z). The orientation in the output Odometry is a yaw estimate computed from consecutive position deltas — there is **no pitch/roll from GNSS**. Downstream, FAST_LIO Mode 2 uses this yaw to set full 6-DOF pose; Mode 3 uses position only, letting LiDAR-IMU handle orientation.

## Demo

**RTK Fusion mode (Mode 3) — stable localization with LiDAR point-cloud mapping:**


![RTK Fusion Demo](docs/RTK_MAPPING.gif)

*Mode 3 — tested in a degraded tunnel scenario. Ground-truth trajectory was converted to simulated RTK 6-DOF measurements for fusion. With RTK fusion enabled, localization remains stable throughout; without fusion, LiDAR-IMU odometry degrades severely in the feature-sparse tunnel environment.*



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

## FAST_LIO RTK Mapping Modes

FAST_LIO supports three operating modes for RTK/GNSS integration:

| Mode | Name | `rtk_mode_en` | `rtk_fuse_en` | RTK DOF | Behavior |
| :---: | --- | :---: | :---: | :---: | --- |
| **1** | Original FAST_LIO | `false` | — | — | Pure LiDAR-IMU odometry. No RTK subscription. |
| **2** | RTK Localization | `true` | `false` | **6-DOF** | RTK pose (position + orientation) directly overrides EKF state each scan. LiDAR matching disabled. Full 6-DOF global localization, but trajectory follows RTK noise. |
| **3** | RTK Fusion | `true` | `true` | **3-DOF** | RTK position only added as measurement residual in IEKF, jointly optimized with LiDAR point-to-plane constraints. Orientation from LiDAR-IMU. Smoother, survives RTK dropout. |


### Mode 3 tuning: `rtk_repeat_n`

| RTK accuracy | Recommended N | Effective R_rtk |
|-------------|:---:|:---:|
| 1 cm | 10 | R_lidar / 10 |
| 2 cm | 3–5 | R_lidar / 3–5 |
| 5 cm | 1 | R_lidar (equal weight) |

### Mode comparison

| | Mode 2 (Override) | Mode 3 (Fusion) |
|---|---|---|
| RTK degrees of freedom | **6-DOF** (position + orientation) | **3-DOF** (position only) |
| Orientation source | RTK (from GNSS heading) | LiDAR-IMU |
| LiDAR scan-matching | ✗ Disabled | ✓ Full IEKF |
| RTK role | Replaces state | Measurement residual |
| Trajectory smoothness | Follows RTK jitter | Smoothed by LiDAR |
| RTK dropout | Mapping stops | Falls back to LiDAR-only |
| Drift without RTK | Immediate | LiDAR-IMU maintains |
| Computational cost | Low | Same as original FAST_LIO |

### Known Issue: Coordinate Frame Misalignment in Mode 3

Mode 3 computes the RTK residual as `rtk_pos − state.pos`, which assumes both vectors live in the same coordinate frame. In practice they do not:

| Frame | Origin | Axes |
|-------|--------|------|
| **RTK ENU** | First GNSS fix (antenna position) | X=East, Y=North, Z=Up |
| **LiDAR `camera_init`** | First LiDAR scan (sensor position) | X=forward, Y=left, Z=up |

The transformation between them is **SE(3)** — a full 6-DOF rigid body displacement (translation from GNSS antenna to LiDAR mounting point, rotation from ENU axes to LiDAR initial heading).

**Why Mode 2 doesn't have this problem:** Mode 2 replaces the EKF state with the RTK pose on the very first scan, effectively rebasing the entire mapping pipeline into the RTK ENU frame. Mode 3 preserves the original FAST_LIO coordinate frame (LiDAR `camera_init`), so the two measurements are in different coordinate systems.

**Impact:** When the vehicle starts with a heading significantly different from East (e.g., pointing North), the residual `rtk_pos − state.pos` mixes components from misaligned axes — the East residual bleeds into the North channel and vice versa. This degrades fusion quality and can cause inconsistent EKF updates.

**Planned fix:** Online SE(2) alignment. On the first valid RTK fix, record both `rtk_pos₀` and `state.pos₀` to estimate a translation offset. Accumulate motion deltas over the first few seconds to estimate the yaw offset between the two frames via:

```
θ_offset = median( atan2(Δrtk_y, Δrtk_x) − atan2(Δekf_y, Δekf_x) )
```

Then transform all subsequent RTK measurements into the LiDAR frame before computing residuals:

```
rtk_cam = R_z(θ_offset)ᵀ · (rtk_pos − T_offset)
innovation = rtk_cam − state.pos
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

### 2. Configure FAST_LIO mode

Edit `FAST_LIO/config/velodyne_rtk.yaml`:

```yaml
rtk:
    rtk_mode_en: true        # enable RTK (false = Mode 1)
    rtk_topic: "/rtk_odom"
    rtk_fuse_en: true        # true = Mode 3 (fusion), false = Mode 2 (override)
    rtk_repeat_n: 5          # Mode 3 only: trust RTK ~5× more than LiDAR
```

### 3. Launch

```bash
# Terminal 1
roslaunch gnss_conversion gnss_conversion.launch

# Terminal 2
roslaunch fast_lio mapping_velodyne_rtk.launch
```

To switch modes, only the YAML config needs changing — no code recompilation required.

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

# gnss_conversion

Converts raw GNSS/RTK `sensor_msgs/NavSatFix` (latitude, longitude, altitude) into local zero-based `nav_msgs/Odometry` for downstream SLAM systems like [FAST_LIO](../FAST_LIO/).

## Pipeline

```
sensor_msgs::NavSatFix  →  gnss_conversion_node  →  nav_msgs::Odometry
(lat, lon, alt)             WGS84→ECEF→ENU         (x, y, z from origin)
                            first fix = origin      heading from motion
```

## Coordinate Conversion

1. **WGS84 → ECEF**: Geodetic coordinates to Earth-Centered Earth-Fixed Cartesian
2. **ECEF → ENU**: Delta from first-fix origin, rotated to local East-North-Up tangent plane
3. **Zero origin**: First valid fix recorded as `(lat₀, lon₀, alt₀)`; all subsequent positions relative to this origin
4. **Heading-from-motion**: Yaw computed from horizontal position delta when speed exceeds `heading_min_speed`

```
rel_pos = R_ENU(origin_lat, origin_lon) · (ECEF(current) − ECEF(origin))
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_rtk_topic` | string | `/rtk_fix` | Input GNSS topic (`sensor_msgs/NavSatFix`) |
| `output_odom_topic` | string | `/rtk_odom` | Output odometry topic (`nav_msgs/Odometry`) |
| `world_frame_id` | string | `odom` | Frame ID for published odometry |
| `child_frame_id` | string | `base_link` | Child frame ID (vehicle body) |
| `publish_tf` | bool | `true` | Broadcast `odom → base_link` TF |
| `reset_on_jump` | bool | `true` | Reset origin on position jump |
| `jump_threshold` | double | `50.0` | Jump detection threshold (meters) |
| `heading_min_speed` | double | `0.5` | Min horizontal speed (m/s) to compute heading |
| `fix_status_min` | int | `0` | Min NavSatFix status (-1=NO_FIX, 0=FIX, 1=SBAS, 2=GBAS) |
| `antenna_offset_x` | double | `0.0` | GNSS antenna offset — forward (meters, base_link frame) |
| `antenna_offset_y` | double | `0.0` | GNSS antenna offset — left (meters, base_link frame) |
| `antenna_offset_z` | double | `0.0` | GNSS antenna offset — up (meters, base_link frame) |

## Key Features

**Jump detection**: If `rel_pos.norm() > jump_threshold`, the origin automatically resets to the current fix. Handles GNSS receiver restarts and coordinate system resets without manual intervention.

**Antenna offset**: Compensates for the GNSS antenna lever-arm relative to `base_link`. The offset is rotated by the current heading estimate before subtraction.

**Heading-from-motion**: Orientation is computed from consecutive position deltas, not from GNSS heading (which is unreliable at low speeds). Below `heading_min_speed`, the last valid heading is held.

**Fix quality gating**: Messages with `status.status <= STATUS_NO_FIX` or below `fix_status_min` are discarded. Zero lat/lon and NaN values are also filtered.

## Usage

```bash
roslaunch gnss_conversion gnss_conversion.launch \
    input_rtk_topic:=/your_gnss_fix \
    output_odom_topic:=/rtk_odom
```

With FAST_LIO RTK mapping:

```bash
# Terminal 1: GNSS conversion
roslaunch gnss_conversion gnss_conversion.launch

# Terminal 2: FAST_LIO (configured with rtk_topic=/rtk_odom)
roslaunch fast_lio mapping_velodyne_rtk.launch
```

## Output Message

The published `nav_msgs/Odometry` contains:

| Field | Content |
|-------|---------|
| `pose.pose.position` | ENU position relative to origin (m) |
| `pose.pose.orientation` | Yaw from heading-from-motion as quaternion |
| `pose.covariance` | Position variance from NavSatFix; orientation set to large uncertainty |
| `twist.twist.linear` | Rotated velocity (if input has velocity) |
| `header.frame_id` | `world_frame_id` (default: `odom`) |
| `child_frame_id` | `child_frame_id` (default: `base_link`) |

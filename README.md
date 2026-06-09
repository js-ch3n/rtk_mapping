# RTK Mapping

LiDAR-inertial mapping with RTK/GNSS absolute positioning. Built on [FAST_LIO](https://github.com/hku-mars/FAST_LIO) with a GNSS conversion layer that transforms raw `NavSatFix` into zero-based ENU odometry.

```
GNSS NavSatFix ──→ gnss_conversion ──→ /rtk/odom ──→ FAST_LIO
(lat/lon/alt)     WGS84→ECEF→ENU      Odometry     LiDAR mapping
                  zero-origin                       w/ absolute pose
```

## Packages

| Package | Role |
|---|---|
| [`FAST_LIO`](FAST_LIO/) | LiDAR-inertial odometry, 3 RTK modes |
| [`gnss_conversion`](gnss_conversion/) | NavSatFix → ENU odometry |

## Build

```bash
cd ~/catkin_ws/src
git clone --recursive <repo_url> rtk_mapping
cd .. && catkin_make && source devel/setup.bash
```

## FAST_LIO RTK Modes

| Mode | `rtk_mode_en` | `rtk_fuse_en` | Behavior |
|------|:---:|:---:|---|
| 1. Original | `false` | — | Pure LiDAR-IMU, no RTK |
| 2. Localization | `true` | `false` | RTK overrides state each scan, LiDAR matching disabled |
| 3. Fusion | `true` | `true` | LiDAR IEKF + independent RTK/yaw EKF corrections |

---

### Mode 3 — RTK Fusion（核心机制）

**目标**：系统可在任意初始姿态下启动，IEKF 在线估计 LiDAR 初始系到 ENU 的刚体变换 `(R_lidar2enu, t_lidar2enu)`，航向由外部 yaw 约束。

#### 状态变量

`state_ikfom` 共 29 自由度（DOF），最后 6 个是 MODE 3 新增的：

```
索引     变量          类型    含义
 0-2    pos           vect3   IMU 在 ENU 下的位置
 3-5    rot           SO3     IMU → ENU 旋转
 6-8    offset_R_L_I  SO3     LiDAR → IMU 旋转外参
 9-11   offset_T_L_I  vect3   LiDAR → IMU 平移外参
12-14   vel           vect3   IMU 速度
15-17   bg            vect3   陀螺零偏
18-20   ba            vect3   加速度计零偏
21-22   grav          S2      重力方向 (2 DOF)
23-25   R_lidar2enu   SO3     ★ LiDAR 初始系 → ENU 旋转
26-28   t_lidar2enu   vect3   ★ LiDAR 初始系原点在 ENU
```

`R_lidar2enu` / `t_lidar2enu` 为静态状态（IMU 传播导数为零），仅由观测方程驱动。

#### 运行流程

```
每帧 LiDAR 扫描:
  │
  ├─ 1. IMU 前传 + 点云去畸变 (UndistortPcl)
  │
  ├─ 2. LiDAR IEKF 迭代 (h_share_model, 12 列, IKFoM 原生)
  │      └─ 残差: 点到拟合平面的距离
  │      └─ 雅可比列: pos, rot, offset_R_L_I, offset_T_L_I (共 12 列)
  │
  ├─ 3. apply_rtk_correction() — RTK 位置 EKF 校正 (3-DOF)
  │      观测: z = rtk_pos (ENU 米)
  │      预测: h(x) = R_lidar2enu · pos + t_lidar2enu
  │      雅可比 (3×29):
  │        列 0-2:   R_lidar2enu                 ∂h/∂pos
  │        列 23-25: -R_lidar2enu · hat(pos)     ∂h/∂R_lidar2enu (SO3 右扰动)
  │        列 26-28: I₃                           ∂h/∂t_lidar2enu
  │      噪声: R_rtk = rtk_pos_cov · I₃  (默认 0.01 m²)
  │
  ├─ 4. apply_yaw_correction() — 航向 EKF 校正 (1-DOF)
  │      观测: z = rtk_yaw (CCW from East, rad)
  │      预测: psi = atan2(M(1,0), M(0,0)),  M = R_lidar2enu · rot
  │      雅可比 (1×29): 完整解析链式法则 ∂atan2/∂(rot, R_lidar2enu)
  │      噪声: R = rtk_yaw_cov  (默认 0.05 rad²)
  │
  └─ 5. state_point = kf.get_x() → map_incremental() → publish
```

#### 关键设计

- **独立校正，不动 IKFoM**：RTK 和 yaw 不在 `h_share_model` 中拼接（IKFoM 硬编码 12 列），而是 IEKF 收敛后各做一次标准 EKF 更新。避免了 IKFoM 库修改和访存越界。
- **各自噪声**：LiDAR 点面匹配用 `LASER_POINT_COV`，RTK 位置用 `rtk_pos_cov`，yaw 用 `rtk_yaw_cov`，不再混用。
- **Joseph 协方差更新**：数值更稳定。
- **`lidar_enu_est_en`**：关闭时雅可比列 23-28 填零，冻结刚体变换估计。

#### 配置

```yaml
# config/mid360_rtk.yaml (或 velodyne_rtk.yaml)
rtk:
    rtk_mode_en: true
    rtk_topic: "/rtk/odom"
    rtk_fuse_en: true           # Mode 3
    rtk_pos_cov: 0.01           # RTK 位置噪声 (m²)
    rtk_yaw_en: true            # 启用 yaw 校正
    rtk_yaw_topic: "/rtk/heading"
    yaw_from_east_cw: true      # 源是 CW from East 时取反
    rtk_yaw_cov: 0.05           # yaw 噪声 (rad²)
    lidar_enu_est_en: true      # 在线估计 R/t
    init_R_lidar2enu: [1,0,0, 0,1,0, 0,0,1]
    init_t_lidar2enu: [0,0,0]
    use_first_rtk_as_origin: false
```

#### 调参

| 参数 | 默认 | 建议 |
|------|------|------|
| `rtk_pos_cov` | 0.01 | RTK 精度高→0.0025, 差→0.04 |
| `rtk_yaw_cov` | 0.05 | 双天线→0.01, 磁力计→0.1 |
| `lidar_enu_est_en` | true | 收敛后可关掉冻结 |

### Mode 2 — RTK Localization

直接读取 `/rtk/odom` 的完整 6-DOF 位姿覆写状态，跳过 LiDAR 匹配。适合快速建图，但 RTK 噪声会直接进入地图。

---

## Usage

### 坐标转换节点

```bash
roslaunch gnss_conversion gnss_conversion_4dof.launch
```

`gnss_conversion_4dof` 需要外部 yaw 源（`/rtk/heading`），输出 4-DOF odometry 到 `/rtk/odom`。若用 heading-from-motion，用 `gnss_conversion.launch`（3-DOF）。

**确认 yaw 方向**：源是标准 CCW from East（朝北 = +90°）则在 yaml 中设 `yaw_from_east_cw: false`。

### FAST_LIO

```bash
# Livox Mid-360
roslaunch fast_lio mapping_mid360_rtk.launch

# Velodyne
roslaunch fast_lio mapping_velodyne_rtk.launch
```

### Mode 2 ↔ Mode 3 切换

改 yaml 一行：

```yaml
rtk_fuse_en: false    # Mode 2
rtk_fuse_en: true     # Mode 3
```

## Coordinate Frames

| Frame | Origin |
|-------|--------|
| `odom` | First GNSS fix (local ENU) |
| `base_link` | Vehicle body |
| `camera_init` | First LiDAR scan |

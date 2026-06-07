# RTK Mapping

基于 [FAST-LIO](https://github.com/hku-mars/FAST_LIO) 改进的激光-惯性- RTK 融合定位与建图方案。

## 主要改进

- **点云去运动畸变**：利用 IMU 预积分和反向传播对原始 LiDAR 点云进行逐点运动补偿，消除高速运动下的点云畸变。
- **RTK 定位融合**：在 FAST-LIO 的紧耦合迭代卡尔曼滤波框架中融合 RTK 全局观测，抑制长时漂移，提供绝对位姿约束。
- **激光建图**：将去畸变后的点云注册到全局地图，输出高精度稠密点云地图。

## 依赖

- Ubuntu >= 18.04 + ROS Melodic/Noetic
- PCL >= 1.8
- Eigen >= 3.3.4
- [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver)（如使用 Livox 系列雷达）

## 构建

```bash
cd ~/catkin_ws/src
git clone <this-repo-url>
cd ..
catkin_make
source devel/setup.bash
```

## 运行

```bash
roslaunch rtk_mapping mapping.launch
```

## 致谢

本项目基于香港大学火星实验室 [FAST-LIO](https://github.com/hku-mars/FAST_LIO) 框架开发，感谢原作者的杰出工作。

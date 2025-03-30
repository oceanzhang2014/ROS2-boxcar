# ROS2-饭盒小车

ROS2控制的小车项目，支持以下功能：

- 运行RPLidar C1激光雷达进行环境感知
- 运行MPU6050 IMU传感器获取运动数据
- 运行电机控制节点控制车辆移动
- 键盘控制功能
- SLAM建图和导航功能

## 安装依赖

```bash
sudo apt install ros-jazzy-rplidar-ros ros-jazzy-slam-toolbox ros-jazzy-cartographer ros-jazzy-cartographer-ros
```

## 如何使用

1. 启动激光雷达：
```bash
ros2 launch raspi_car lidar.launch.py
```

2. 启动IMU节点：
```bash
ros2 run raspi_car mpu6050_node.py
```

3. 启动电机控制节点：
```bash
ros2 run raspi_car motor_controller_node.py
```

4. 启动键盘控制节点：
```bash
ros2 run raspi_car keyboard_controller_node.py
```

## SLAM建图功能

有两种SLAM建图方式可供选择：

### 1. 使用Cartographer（推荐）

```bash
# 直接启动Cartographer SLAM功能（包含激光雷达和底盘控制）
./src/raspi_car/scripts/start_cartographer.sh
```

### 2. 使用SLAM Toolbox

```bash
# 直接启动SLAM Toolbox功能（包含激光雷达和底盘控制）
./src/raspi_car/scripts/start_slam_mapping.sh
```

### 控制小车建图

通过键盘控制小车移动，实现环境建图：
```bash
# 在另一个终端中启动键盘控制
ros2 run raspi_car keyboard_controller_node.py
```

### 保存地图

建图完成后保存地图：
```bash
# 保存地图（自动使用当前时间作为文件名）
./src/raspi_car/scripts/save_map.sh
# 或指定地图名称
./src/raspi_car/scripts/save_map.sh my_map_name
```

### 建图优化说明

如果在建图过程中遇到以下问题：
- 小车转弯但地图上显示不正确
- 直行时地图出现漂移
- 回到原点时位置不匹配

可以尝试以下优化方法：
1. 确保IMU（MPU6050）安装牢固，减少振动
2. 小车在转弯时速度放慢，让传感器有足够时间捕获环境变化
3. 在转弯后直行一段距离，帮助算法识别特征点
4. 避免在相似度高的环境中建图（如长走廊、对称房间）

当前配置已优化以使用IMU数据提高转弯精度。

## 硬件连接

- 激光雷达连接到USB端口，映射为/dev/ttyUSB0
- MPU6050通过I2C连接到树莓派
- 电机驱动通过GPIO针脚连接

## 常用命令

```bash
# 编译项目
cd ~/raspi_car_ws && colcon build --packages-select raspi_car

# 设置环境
source ~/raspi_car_ws/install/setup.bash

# 设置启动文件权限
cd ~/raspi_car_ws/src/raspi_car/launch && chmod +x odom_fusion.launch.py car_complete.launch.py slam_mapping.launch.py cartographer_slam.launch.py

# 启动小车基础功能
source ~/raspi_car_ws/install/setup.bash && ros2 launch raspi_car car_base.launch.py

# 启动激光雷达
source ~/raspi_car_ws/install/setup.bash && ros2 launch raspi_car lidar.launch.py

# 启动RViz可视化
source ~/raspi_car_ws/install/setup.bash && ~/raspi_car_ws/install/raspi_car/share/raspi_car/scripts/start_rviz.sh

# 启动SLAM建图 (使用slam_toolbox)
source ~/raspi_car_ws/install/setup.bash && ros2 launch raspi_car slam_mapping.launch.py

# 启动SLAM建图 (使用cartographer)
source ~/raspi_car_ws/install/setup.bash && ros2 launch raspi_car cartographer_slam.launch.py

# 启动Cartographer地图可视化
source ~/raspi_car_ws/install/setup.bash && ./src/raspi_car/scripts/rviz_cartographer.sh
``` 


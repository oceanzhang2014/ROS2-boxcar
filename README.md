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
source ~/ROS2-boxcar/install/setup.bash && ros2 launch raspi_car lidar.launch.py
```

2. 启动IMU节点：
```bash
source ~/ROS2-boxcar/install/setup.bash && ros2 run raspi_car mpu6050_node.py
```

3. 启动电机控制节点：
```bash
source ~/ROS2-boxcar/install/setup.bash && ros2 run raspi_car motor_controller_node.py
```

4. 启动键盘控制节点：
```bash
source ~/ROS2-boxcar/install/setup.bash && ros2 run raspi_car keyboard_controller_node.py
```

## SLAM建图功能

有两种SLAM建图方式可供选择：

### 1. 使用Cartographer（推荐）

```bash
# 直接启动Cartographer SLAM功能（包含激光雷达、IMU和底盘控制）
ros2 launch raspi_car cartographer_slam.launch.py
```

这个启动文件会：
- 初始化机器人状态发布
- 启动IMU传感器（MPU6050）
- 初始化基于IMU的里程计融合
- 启动RPLidar激光雷达
- 启动Cartographer SLAM节点
- 生成占用栅格地图

启动后，可以使用键盘控制节点来控制小车移动，实现环境建图：

```bash
# 在另一个终端中启动键盘控制
ros2 run raspi_car keyboard_controller_node.py
```

### 2. 使用SLAM Toolbox

```bash
# 直接启动SLAM Toolbox功能（包含激光雷达和底盘控制）
ros2 launch raspi_car slam_mapping.launch.py
```

或者使用脚本启动：

```bash
./src/raspi_car/scripts/start_slam_mapping.sh
```

### 控制小车建图

通过键盘控制小车移动，实现环境建图：
```bash
# 在另一个终端中启动键盘控制
ros2 run raspi_car keyboard_controller_node.py
```

### 可视化建图效果

要查看建图效果，可以使用RViz：

```bash
# 启动RViz可视化Cartographer建图效果
source install/setup.bash && ./src/raspi_car/scripts/rviz_cartographer.sh
```

或者在远程电脑上启动RViz：

```bash
# 设置X11连接（替换YOUR_WINDOWS_IP为Windows电脑的IP地址）
./src/raspi_car/scripts/setup_x11_display.sh YOUR_WINDOWS_IP

# 启动RViz
./src/raspi_car/scripts/start_remote_rviz.sh YOUR_WINDOWS_IP
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

当前配置已优化以使用IMU数据提高转弯精度。Cartographer SLAM使用了稳定的激光雷达坐标系和IMU里程计融合，可以提供更加精确的建图结果。

## 硬件连接

- 激光雷达连接到USB端口，映射为/dev/ttyUSB0
- MPU6050通过I2C连接到树莓派
- 电机驱动通过GPIO针脚连接

## 常用命令

```bash
# 编译项目
cd ~/ROS2-boxcar && colcon build --packages-select raspi_car

# 设置环境
source ~/ROS2-boxcar/install/setup.bash

# 设置启动文件权限
cd ~/ROS2-boxcar/src/raspi_car/launch && chmod +x *.launch.py

# 启动小车基础功能
source ~/ROS2-boxcar/install/setup.bash && ros2 launch raspi_car car_base.launch.py

# 启动激光雷达
source ~/ROS2-boxcar/install/setup.bash && ros2 launch raspi_car lidar.launch.py

# 启动RViz可视化
source ~/ROS2-boxcar/install/setup.bash && ./src/raspi_car/scripts/start_rviz.sh

# 启动SLAM建图 (使用slam_toolbox)
source ~/ROS2-boxcar/install/setup.bash && ros2 launch raspi_car slam_mapping.launch.py

# 启动SLAM建图 (使用cartographer)
source ~/ROS2-boxcar/install/setup.bash && ros2 launch raspi_car cartographer_slam.launch.py

# 启动Cartographer地图可视化
source ~/ROS2-boxcar/install/setup.bash && ./src/raspi_car/scripts/rviz_cartographer.sh

# 保存地图（自动使用当前时间作为文件名）
./src/raspi_car/scripts/save_map.sh

# 或指定地图名称
./src/raspi_car/scripts/save_map.sh my_map_name
``` 

## 故障排除

如果在使用过程中遇到问题，请参考详细的故障排除指南：
```bash
# 查看SLAM建图详细文档
less src/raspi_car/README_SLAM.md
```

主要故障排除技巧：
1. 确保所有硬件连接正确，特别是激光雷达USB端口
2. 检查IMU是否校准成功
3. 验证TF转换树是否正确设置
4. 如果地图显示不正确，尝试降低小车速度提高建图质量


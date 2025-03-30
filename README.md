# 树莓派ROS 2小车项目

这个项目实现了一个基于ROS 2的树莓派智能小车，具有SLAM建图、自主导航、遥控操作等功能。项目使用树莓派作为主控制器，配备激光雷达、IMU传感器和电机驱动，构建了一个功能完整的移动机器人平台。

## 项目特点

- 基于ROS 2 Jazzy实现
- 支持SLAM自主建图（SLAM Toolbox和Cartographer双引擎）
- 支持Navigation2自主导航
- 通过键盘远程遥控
- MPU6050姿态检测与监控
- 支持头显示模式与无GUI模式
- 优化的远程可视化支持

## 硬件要求

- 树莓派（建议4B或更高版本）
- 电机驱动板（如L298N）
- 激光雷达（如RPLIDAR）
- MPU6050 IMU传感器
- 小车底盘与电机
- 电源（锂电池或其他）

## 软件依赖

- Ubuntu 24.04 (Jammy Jellyfish)
- ROS 2 Jazzy
- SLAM Toolbox
- Cartographer SLAM
- Navigation2
- 各种依赖库

## 安装与配置

### 1. 安装ROS 2 Jazzy

按照[ROS 2 Jazzy官方安装指南](https://docs.ros.org/en/jazzy/Installation.html)安装ROS 2。

### 2. 安装必要的依赖

```bash
sudo apt update
sudo apt install -y python3-pip python3-rosdep python3-colcon-common-extensions
sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup ros-jazzy-navigation2
sudo apt install -y ros-jazzy-cartographer ros-jazzy-cartographer-ros
sudo apt install -y ros-jazzy-tf2-ros ros-jazzy-tf2-tools
pip3 install smbus2 numpy
```

### 3. 克隆代码库并编译

```bash
mkdir -p ~/raspi_car_ws/src
cd ~/raspi_car_ws/src
# 克隆此代码库
git clone https://github.com/your_username/raspi_car.git

# 编译工作空间
cd ~/raspi_car_ws
colcon build --symlink-install
```

### 4. 配置硬件接口权限

```bash
# 配置I2C权限
sudo usermod -a -G i2c $USER
sudo chmod 666 /dev/i2c-1

# 配置串口权限（用于激光雷达）
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0

# 配置GPIO权限（如果使用GPIO接口）
sudo usermod -a -G gpio $USER
```

## 主要功能与使用命令

### MPU6050 IMU传感器节点

MPU6050节点负责读取IMU传感器数据，提供姿态信息。
cd ~/raspi_car_ws && colcon build --packages-select raspi_car
```bash
# 运行MPU6050节点
source install/setup.bash
ros2 run raspi_car mpu6050_node.py
source install/setup.bash && python3 src/raspi_car/scripts/mpu6050_node.py
source ~/raspi_car_ws/install/setup.bash && ros2 topic echo /imu/data_raw --once
# 查看IMU数据
ros2 run raspi_car mpu6050_subscriber.py

# 测试MPU6050传感器连接
python3 src/raspi_car/scripts/test_mpu6050.py
```

### 电机控制节点

电机控制节点接收速度命令并驱动小车电机。

```bash
# 运行电机控制节点
ros2 run raspi_car motor_controller_node.py

# 发送测试速度命令
# 全速前进
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 左转
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"

# 停止
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 键盘控制节点

键盘控制允许通过SSH远程控制小车。

```bash
# 运行键盘控制节点
ros2 run raspi_car keyboard_controller_node.py
```

### 激光雷达节点

激光雷达节点读取激光雷达数据，发布扫描结果。

```bash
# 运行激光雷达节点
ros2 launch raspi_car lidar.launch.py 
```

### TF发布节点

TF发布节点负责维护坐标系变换关系。

```bash
# 运行TF发布节点
ros2 run raspi_car tf_publisher_node.py
```

## 远程可视化功能

本项目提供四种方式实现远程可视化：Mesa软件渲染（新增）、XLaunch优化版、标准X11模式和VNC模式，以及无GUI的服务器模式。

### VSCode使用特别说明

如果您使用VSCode远程SSH连接树莓派，需要特别设置才能使用图形界面：

1. **运行自动配置脚本**：
   ```bash
   cd ~/raspi_car_ws
   source install/setup.bash
   ros2 run raspi_car setup_x11_vscode.sh
   ```
   这个脚本会自动：
   - 检测网络环境
   - 设置DISPLAY环境变量
   - 将设置添加到~/.bashrc
   - 测试X11连接
   - 提供测试命令

2. **在Windows上运行XServer**：
   - 从[此处](https://sourceforge.net/projects/vcxsrv/)下载并安装VcXsrv
   - 启动XLaunch
   - 选择"Multiple Windows"
   - 选择"Start no client"
   - **必须勾选"Disable access control"**
   - 保存配置以便后续使用

3. **检查OpenGL环境**：
   ```bash
   ros2 run raspi_car check_opengl.sh
   ```

4. **运行应用**：
   ```bash
   ros2 launch raspi_car slam_xlaunch.launch.py
   ros2 launch raspi_car navigation_xlaunch.launch.py
   ```

### 1. Mesa软件渲染（推荐）

Mesa软件渲染解决方案专为解决X11转发RViz崩溃问题设计，通过使用软件渲染代替硬件加速，确保在任何环境下的稳定性。

#### 1.1 使用Mesa软件渲染启动

```bash
# 先用SSH连接树莓派（需带-X参数）
ssh -X ocean@192.168.0.13  # 替换为您的树莓派IP
cd ~/raspi_car_ws && colcon build --packages-select raspi_car


#### 1.2 单独启动RViz2（使用Mesa软件渲染）

如果需要单独启动RViz2进行测试或调试：

```bash
cd ~/raspi_car_ws
source install/setup.bash
ros2 run raspi_car scripts/start_rviz_mesa.sh path/to/your/config.rviz
```

#### 1.3 测试Mesa软件渲染是否工作

我们提供了一个简单的测试脚本，可以验证Mesa软件渲染是否正常工作：

```bash
cd ~/raspi_car_ws
source install/setup.bash
ros2 run raspi_car scripts/test_mesa_rviz.sh
```

这个脚本会尝试使用Mesa软件渲染启动RViz2，并提供详细的问题排查步骤（如果测试失败）。

### 2. XLaunch优化版

XLaunch优化版专为解决Windows上使用X11转发RViz崩溃问题而设计，提供最佳的远程体验。

#### 1.1 在Windows上设置XLaunch

1. 从[此处](https://sourceforge.net/projects/vcxsrv/)下载并安装VcXsrv
2. 启动XLaunch：
   - 选择"Multiple Windows"
   - Display number设为0
   - 选择"Start no client"
   - **必须勾选"Disable access control"**
   - 保存配置以便后续使用

#### 1.2 优化X11环境并启动

在树莓派上使用我们提供的优化脚本：

```bash
# 先用SSH连接树莓派（无需-X参数）
ssh ocean@192.168.0.13  # 替换为您的树莓派IP

# 运行XLaunch优化脚本
cd ~/raspi_car_ws
source install/setup.bash
bash src/raspi_car/scripts/optimize_xlaunch.sh 192.168.0.4  # 替换为Windows IP
```
source install/setup.bash && ros2 launch raspi_car car_lidar_rviz.launch.py
或者直接启动优化版本的SLAM或导航：

```bash
# 启动SLAM (XLaunch优化版)
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car slam_xlaunch.launch.py windows_ip:=192.168.0.4

# 启动导航 (XLaunch优化版)
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car navigation_xlaunch.launch.py windows_ip:=192.168.0.100
```

### 3. 标准X11转发

标准X11转发方式适用于不需要特殊优化的情况：

```bash
# 1. 在Windows上启动VcXsrv (同上述设置)

# 2. 使用X11转发连接树莓派
ssh -X ocean@192.168.0.13

# 3. 启动SLAM或导航
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car slam.launch.py windows_ip:=192.168.0.100
```

### 4. VNC模式

VNC模式不依赖于X11转发，适用于任何环境：

```bash
# 1. 设置VNC服务器
cd ~/raspi_car_ws
bash src/raspi_car/scripts/setup_vnc.sh

# 2. 启动SLAM或导航 (VNC模式)
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car slam_vnc.launch.py
```

然后使用任何VNC客户端（如VNC Viewer）连接到树莓派的5900端口。

### 5. 无GUI模式

如果不需要可视化界面，适合资源受限的设备：

```bash
# 启动SLAM (无GUI)
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car slam_headless.launch.py

# 在另一个终端控制小车
ros2 run raspi_car keyboard_controller_node.py

# 完成后保存地图
ros2 run nav2_map_server map_saver_cli -f src/raspi_car/maps/map
```

## SLAM系统

本项目提供两种SLAM引擎实现，可根据实际需求选择：

### 1. Cartographer SLAM（推荐）

Cartographer是Google开发的高性能SLAM系统，具有出色的精度和效率，特别适合资源受限的设备。

#### 1.1 Cartographer SLAM特点
- 资源占用低，特别适合树莓派等嵌入式设备
- 实时性能好，建图速度快
- 闭环检测能力强
- 稳定的位姿估计

#### 1.2 使用方法

**标准版：**
```bash
cd ~/raspi_car_ws && colcon build --packages-select raspi_car
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car cartographer_slam.launch.py
```

**XLaunch优化版：**
```bash
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car cartographer_slam_xlaunch.launch.py windows_ip:=192.168.0.4
```

#### 1.3 保存地图

在完成建图后，保存Cartographer地图：

```bash
# 保存为pbstream格式（Cartographer原始格式）
cd ~/raspi_car_ws
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename=/home/ocean/raspi_car_ws/src/raspi_car/maps/map.pbstream \
  -map_filestem=/home/ocean/raspi_car_ws/src/raspi_car/maps/map
```

这将生成标准的ROS 2地图文件（`.yaml`和`.pgm`），可用于导航。

#### 1.4 基于Cartographer的导航

```bash
# 标准版
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car navigation_cartographer.launch.py

# XLaunch优化版
ros2 launch raspi_car navigation_cartographer_xlaunch.launch.py windows_ip:=192.168.0.4
```

### 2. SLAM Toolbox

SLAM Toolbox是ROS 2默认的SLAM系统，提供了完整的功能集。

#### 2.1 使用方法

**标准版：**
```bash
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car slam.launch.py
```

**XLaunch优化版：**
```bash
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car slam_xlaunch.launch.py windows_ip:=192.168.0.4
```

#### 2.2 保存地图

```bash
cd ~/raspi_car_ws
ros2 run nav2_map_server map_saver_cli -f src/raspi_car/maps/map
```

## SLAM建图流程

1. 启动SLAM（选择Cartographer或SLAM Toolbox任一方式）：
   ```bash
   # 使用Cartographer SLAM（推荐）
   ros2 launch raspi_car cartographer_slam.launch.py
   # 或使用优化版
   ros2 launch raspi_car cartographer_slam_xlaunch.launch.py windows_ip:=192.168.0.4
   
   # 或者使用SLAM Toolbox
   ros2 launch raspi_car slam.launch.py
   # 或使用优化版
   ros2 launch raspi_car slam_xlaunch.launch.py windows_ip:=192.168.0.4
   ```
   
2. 在另一个终端启动键盘控制：
   ```bash
   ros2 run raspi_car keyboard_controller_node.py
   ```
   
3. 使用键盘控制小车在环境中移动，激光雷达会不断扫描周围环境创建地图

4. 地图探索完成后，保存地图：
   ```bash
   # 使用SLAM Toolbox时
   ros2 run nav2_map_server map_saver_cli -f src/raspi_car/maps/map
   
   # 使用Cartographer时
   ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
     -pbstream_filename=/home/ocean/raspi_car_ws/src/raspi_car/maps/map.pbstream \
     -map_filestem=/home/ocean/raspi_car_ws/src/raspi_car/maps/map
   ```

## 导航功能使用

1. 确保已完成SLAM建图并保存地图
2. 启动导航（选择上述任一方式）
3. 在RViz界面中：
   - 使用"2D Pose Estimate"设置初始位置
   - 使用"2D Nav Goal"设置导航目标

或者通过命令行发送导航目标：
```bash
ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

## 故障排除

### X11/RViz相关问题

如果使用X11转发时RViz崩溃：

1. **使用Mesa软件渲染（最稳定）**：
   ```bash
   # 确保先安装Mesa软件渲染相关包
   sudo apt-get install -y mesa-utils libosmesa6 libopengl0
   
   # 通过修改后的启动文件启动
   ros2 launch raspi_car slam_xlaunch.launch.py
   ```

2. **VSCode用户的特殊设置**：
   ```bash
   # 运行特别为VSCode用户设计的配置脚本
   ros2 run raspi_car setup_x11_vscode.sh
   
   # 重新加载环境变量
   source ~/.bashrc
   
   # 然后启动应用
   ros2 launch raspi_car slam_xlaunch.launch.py
   ```

3. **使用优化版XLaunch**：
   ```bash
   bash src/raspi_car/scripts/optimize_xlaunch.sh 192.168.0.100
   ```

4. **检查以下常见错误**：
   - Windows防火墙是否阻止VcXsrv
   - XLaunch是否勾选了"Disable access control"
   - Windows IP地址是否正确

5. **如果仍有问题，尝试VNC模式或无GUI模式**

### 其他问题

1. **激光雷达无法连接**
   - 检查权限：`sudo chmod 666 /dev/ttyUSB0`
   - 验证设备名称：`ls -l /dev/ttyUSB*`
   - 检查波特率是否正确

2. **MPU6050无法读取**
   - 检查I2C连接：`sudo i2cdetect -y 1`
   - 验证接线是否正确
   - 检查I2C权限

3. **电机控制问题**
   - 检查GPIO配置
   - 测试PWM功能
   - 验证电源是否足够

## 目录结构

```
raspi_car_ws/
└── src/
    └── raspi_car/
        ├── raspi_car/           # Python包
        ├── scripts/             # 可执行脚本
        │   ├── mpu6050_node.py
        │   ├── motor_controller_node.py
        │   ├── keyboard_controller_node.py
        │   ├── lidar_driver_node.py
        │   ├── tf_publisher_node.py
        │   ├── optimize_xlaunch.sh    # XLaunch优化脚本
        │   └── start_rviz_mesa.sh     # Mesa软件渲染启动脚本
        ├── launch/              # 启动文件
        │   ├── slam.launch.py              # SLAM Toolbox启动文件
        │   ├── slam_headless.launch.py     # 无GUI版SLAM Toolbox
        │   ├── slam_xlaunch.launch.py      # XLaunch优化版SLAM Toolbox
        │   ├── cartographer_slam.launch.py      # Cartographer SLAM
        │   ├── cartographer_slam_xlaunch.launch.py  # XLaunch优化版Cartographer
        │   ├── navigation.launch.py              # 基于SLAM Toolbox的导航
        │   ├── navigation_headless.launch.py     # 无GUI版导航
        │   ├── navigation_xlaunch.launch.py      # XLaunch优化版导航
        │   ├── navigation_cartographer.launch.py      # 基于Cartographer的导航
        │   └── navigation_cartographer_xlaunch.launch.py  # XLaunch优化版Cartographer导航
        ├── config/              # 配置文件
        │   ├── slam.rviz                  # SLAM可视化配置
        │   ├── navigation.rviz            # 导航可视化配置
        │   ├── slam_params.yaml           # SLAM Toolbox参数
        │   ├── nav2_params.yaml           # Navigation2参数
        │   └── raspi_car_cartographer.lua # Cartographer配置文件
        └── maps/                # 地图存储目录
            ├── map.pgm          # 栅格地图图像
            ├── map.yaml         # 栅格地图配置
            └── map.pbstream     # Cartographer原始格式地图
```

## 许可证

此项目使用[MIT许可证](LICENSE)。

## 贡献

欢迎贡献代码和提出建议！请提交issue或pull request。

## 致谢

- ROS 2社区
- Navigation2团队
- SLAM Toolbox开发者
- Cartographer团队（Google）
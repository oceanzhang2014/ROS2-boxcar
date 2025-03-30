# 树莓派ROS 2 SLAM建图指南

本文档记录使用RPLIDAR C1激光雷达和Cartographer进行SLAM建图的主要命令和步骤。

## 环境准备

### 1. 安装依赖

```bash
# 安装gpiozero和RPi.GPIO库（用于电机控制）
sudo apt install -y python3-gpiozero python3-rpi.gpio

# 安装Mesa软件渲染所需包（用于远程X11显示）
sudo apt install -y mesa-utils libosmesa6 libopengl0
```

### 2. 设置设备权限

```bash
# 设置RPLIDAR C1设备权限
sudo chmod 666 /dev/rplidar
# 或者
sudo chmod 666 /dev/ttyUSB0

# 设置GPIO权限（用于电机控制）
sudo chmod 666 /dev/gpiomem
```

## 远程X11显示设置

在Windows电脑上:

1. 下载并安装VcXsrv: https://sourceforge.net/projects/vcxsrv/
2. 启动XLaunch:
   - 选择"Multiple Windows"
   - 设置Display number为0
   - 选择"Start no client"
   - **必须勾选"Disable access control"**
   - 点击"Next"并完成设置

在树莓派上:

```bash
# 设置X11环境（替换YOUR_WINDOWS_IP为Windows电脑的IP地址）
~/raspi_car_ws/src/raspi_car/scripts/setup_x11_display.sh YOUR_WINDOWS_IP
```

## 运行Cartographer SLAM建图

### 方法0: 分步式测试方法（最佳调试方式）

这种方法将激光雷达启动和RViz显示分开，便于排查问题：

```bash
# 步骤1: 先在一个终端中启动激光雷达和TF
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car lidar_only.launch.py serial_port:=/dev/rplidar

# 步骤2: 在另一个终端启动远程RViz显示
cd ~/raspi_car_ws
source install/setup.bash
~/raspi_car_ws/src/raspi_car/scripts/start_remote_rviz.sh 192.168.0.4
```

### 方法1: 使用简化版启动文件（推荐）

这是最新的简化版启动方法，解决了所有已知问题:

```bash
# 终端1: 启动SLAM核心组件
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car simple_slam.launch.py serial_port:=/dev/rplidar

# 终端2: 在另一个终端启动远程RViz (替换YOUR_WINDOWS_IP为Windows电脑IP地址)
cd ~/raspi_car_ws
source install/setup.bash
~/raspi_car_ws/src/raspi_car/scripts/start_remote_rviz.sh YOUR_WINDOWS_IP

# 终端3: 启动键盘控制
cd ~/raspi_car_ws
source install/setup.bash
cd ~/raspi_car_ws/src/raspi_car/scripts
python3 keyboard_controller_node.py
```

### 方法2: 使用改进版启动文件

```bash
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car improved_slam.launch.py serial_port:=/dev/rplidar windows_ip:=YOUR_WINDOWS_IP
```

这个启动文件会:
- 自动设置X11环境
- 启动RPLIDAR C1激光雷达节点
- 启动Cartographer SLAM节点
- 启动电机控制节点
- 在Windows电脑上显示RViz界面

### 方法3: 分步启动各组件

```bash
# 终端1: 启动激光雷达节点
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car lidar.launch.py serial_port:=/dev/rplidar

# 终端2: 启动Cartographer SLAM
cd ~/raspi_car_ws
source install/setup.bash
ros2 run cartographer_ros cartographer_node -configuration_directory ~/raspi_car_ws/install/raspi_car/share/raspi_car/config -configuration_basename raspi_car_cartographer.lua

# 终端3: 启动占用栅格地图生成节点
cd ~/raspi_car_ws
source install/setup.bash
ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05

# 终端4: 启动电机控制节点
cd ~/raspi_car_ws
source install/setup.bash
~/raspi_car_ws/src/raspi_car/scripts/start_motor_controller.sh

# 终端5: 启动键盘控制
cd ~/raspi_car_ws
source install/setup.bash
cd ~/raspi_car_ws/src/raspi_car/scripts
python3 keyboard_controller_node.py

# 终端6: 启动远程RViz (在设置好X11环境后)
cd ~/raspi_car_ws
source install/setup.bash
DISPLAY=YOUR_WINDOWS_IP:0.0 LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3 ros2 run rviz2 rviz2 -d ~/raspi_car_ws/install/raspi_car/share/raspi_car/config/cartographer.rviz
```

## 仅测试雷达显示

如果想要单独测试雷达数据是否能在RViz中正确显示，可以使用专门的测试启动文件：

```bash
cd ~/raspi_car_ws
source install/setup.bash
ros2 launch raspi_car test_lidar_viz.launch.py serial_port:=/dev/rplidar windows_ip:=192.168.0.4
```

这个启动文件会：
- 只启动雷达节点和必要的TF转换
- 在远程电脑上显示RViz
- 不包含Cartographer SLAM和电机控制等复杂功能

## 保存地图

在完成地图构建后，可以使用以下命令保存地图:

```bash
# 保存为Cartographer原始格式
cd ~/raspi_car_ws
source install/setup.bash
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename=/home/ocean/raspi_car_ws/src/raspi_car/maps/map.pbstream \
  -map_filestem=/home/ocean/raspi_car_ws/src/raspi_car/maps/map
```

## 键盘控制说明

使用以下键位控制小车移动:
- `w`: 前进
- `s`: 后退
- `a`: 左转
- `d`: 右转
- `q`: 左前
- `e`: 右前
- `z`: 左后
- `c`: 右后
- 空格: 停止
- `Ctrl+C`: 退出控制

## 故障排除

### 启动文件错误问题

如果遇到以下错误:
```
TypeError: can only concatenate str (not "LaunchConfiguration") to str
```

这是因为在ROS 2启动文件中，不能直接将 `LaunchConfiguration` 对象与字符串相加。解决方法有两种:

1. **使用PythonExpression**:
   ```python
   from launch.substitutions import PythonExpression
   
   # 正确用法
   cmd=[
       'bash', '-c',
       PythonExpression([
           "'DISPLAY=" + windows_ip + ":0.0 ...'"
       ])
   ]
   ```

2. **分步启动**:
   使用我们提供的分步式测试方法，先启动激光雷达，再单独启动RViz:
   ```bash
   # 步骤1: 启动激光雷达
   ros2 launch raspi_car lidar_only.launch.py
   
   # 步骤2: 另一个终端启动RViz
   ~/raspi_car_ws/src/raspi_car/scripts/start_remote_rviz.sh 192.168.0.4
   ```

### RViz无法显示雷达数据问题

如果在RViz中看不到雷达扫描数据，可能存在以下问题：

1. **帧ID不匹配问题**:
   RViz配置中使用的帧ID与实际发布的帧ID不一致。检查并修复方法：
   
   ```bash
   # 查看雷达消息的帧ID
   ros2 topic echo /scan --once | grep frame_id
   
   # 查看TF树中的帧
   ros2 topic echo /tf_static --once | grep frame_id
   ```
   
   **解决方法**：使用 `test_lidar_viz.launch.py` 启动文件，它会添加所有必要的TF转换，确保帧ID匹配。

2. **TF树不完整问题**:
   可能缺少重要的TF转换链，如 `map` -> `odom` -> `base_link` -> `laser`。

   **解决方法**：使用我们提供的测试启动文件，它会创建完整的TF转换链。

3. **时间同步问题**:
   如果雷达数据的时间戳与ROS系统时间不同步，可能导致显示问题。

   **解决方法**：确保不使用仿真时间（`use_sim_time:=false`）。

4. **X11连接问题**:
   RViz可能无法正确连接到X服务器。

   **解决方法**：使用增强版的启动脚本：
   ```bash
   ~/raspi_car_ws/src/raspi_car/scripts/start_remote_rviz.sh YOUR_WINDOWS_IP
   ```
   此脚本会进行连接测试并提供详细的调试信息。

### 其他常见问题

1. **RViz无法显示**: 
   - 确保Windows上的XLaunch运行时勾选了"Disable access control"
   - 检查Windows防火墙是否阻止了TCP端口6000
   - 使用`~/raspi_car_ws/src/raspi_car/scripts/setup_x11_display.sh YOUR_WINDOWS_IP`测试X11连接

2. **电机控制节点无法启动**:
   - 检查GPIO权限: `sudo chmod 666 /dev/gpiomem`
   - 检查是否已安装GPIO库: `sudo apt install python3-gpiozero python3-rpi.gpio`

3. **激光雷达无法连接**:
   - 检查串口设备是否存在: `ls -l /dev/rplidar`
   - 设置串口权限: `sudo chmod 666 /dev/rplidar`
   - 如果设备名称不同，请相应修改参数 
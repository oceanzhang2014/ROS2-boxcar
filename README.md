# ROS2-饭盒小车

ROS2控制的小车项目，支持以下功能：

- 运行RPLidar C1激光雷达进行环境感知
- 运行MPU6050 IMU传感器获取运动数据
- 运行电机控制节点控制车辆移动
- 键盘控制功能

## 安装依赖

```bash
sudo apt install ros-jazzy-rplidar-ros
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

## 硬件连接

- 激光雷达连接到USB端口，映射为/dev/rplidar
- MPU6050通过I2C连接到树莓派
- 电机驱动通过GPIO针脚连接
cd ~/raspi_car_ws && colcon build --packages-select raspi_car
source ~/raspi_car_ws/install/setup.bash
cd ~/raspi_car_ws/src/raspi_car/launch && chmod +x odom_fusion.launch.py car_complete.launch.py

source ~/raspi_car_ws/install/setup.bash && ros2 launch raspi_car car_base.launch.py
source ~/raspi_car_ws/install/setup.bash && ros2 launch raspi_car lidar.launch.py
source ~/raspi_car_ws/install/setup.bash && ~/raspi_car_ws/install/raspi_car/share/raspi_car/scripts/start_rviz.sh 
#!/bin/bash

# 设置ROS2环境
source ~/raspi_car_ws/install/setup.bash

# 启动Cartographer SLAM
ros2 launch raspi_car cartographer_slam.launch.py "$@" 
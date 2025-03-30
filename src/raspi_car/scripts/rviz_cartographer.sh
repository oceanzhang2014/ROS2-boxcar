#!/bin/bash

# 设置ROS2环境
source ~/raspi_car_ws/install/setup.bash

# 定义RViz配置文件
RVIZ_CONFIG=$(ros2 pkg prefix raspi_car)/share/raspi_car/config/cartographer.rviz

# 获取本地显示服务器IP
# 如果您知道正确的显示IP，可以直接设置
# 比如: DISPLAY_IP=192.168.0.4
DISPLAY_IP=${1:-"192.168.0.4"}

echo "启动RViz查看Cartographer地图..."
echo "使用显示服务器: $DISPLAY_IP:0.0"
echo "使用配置文件: $RVIZ_CONFIG"

# 使用环境变量启动RViz
export DISPLAY=$DISPLAY_IP:0.0
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export XDG_RUNTIME_DIR=/run/user/$(id -u)

# 启动RViz
ros2 run rviz2 rviz2 -d $RVIZ_CONFIG 
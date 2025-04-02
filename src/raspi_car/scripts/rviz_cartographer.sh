#!/bin/bash

# 设置ROS2环境
source ~/ROS2-boxcar/install/setup.bash

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
# LIBGL_ALWAYS_SOFTWARE=1 强制使用软件渲染，解决树莓派上的OpenGL问题
# MESA_GL_VERSION_OVERRIDE=3.3 设置Mesa OpenGL版本，确保兼容性
export DISPLAY=$DISPLAY_IP:0.0
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export XDG_RUNTIME_DIR=/run/user/$(id -u)

# 提示信息
echo "如果RViz无法启动，请检查:"
echo "1. 确保Windows上的XLaunch已运行并勾选'Disable access control'"
echo "2. 确保Windows防火墙未阻止TCP端口6000"
echo "3. 使用正确的IP地址: ${DISPLAY_IP}"

# 启动RViz
ros2 run rviz2 rviz2 -d $RVIZ_CONFIG 
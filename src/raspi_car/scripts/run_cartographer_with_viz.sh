#!/bin/bash

# 设置ROS2环境
source ~/ROS2-boxcar/install/setup.bash

echo "启动Cartographer SLAM建图 (带TF修复，解决RViz显示问题)..."

# 先检查cartographer_node是否已经运行
if pgrep -f "cartographer_node" > /dev/null; then
    echo "检测到cartographer_node已经在运行。先关闭它..."
    pkill -f "cartographer_node"
    sleep 2
fi

# 先检查tf_fix_node是否已经运行
if pgrep -f "tf_fix_node" > /dev/null; then
    echo "检测到tf_fix_node已经在运行。先关闭它..."
    pkill -f "tf_fix_node"
    sleep 2
fi

# 启动包含TF修复的Cartographer SLAM
ros2 launch raspi_car cartographer_slam_viz.launch.py 
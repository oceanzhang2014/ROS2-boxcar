#!/bin/bash

# 设置ROS2环境
source ~/raspi_car_ws/install/setup.bash

# 获取包路径
PARAMS_FILE=$(ros2 pkg prefix raspi_car)/share/raspi_car/config/slam_params.yaml

echo "使用参数文件: $PARAMS_FILE"

# 仅启动SLAM Toolbox节点
ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p use_sim_time:=false --params-file $PARAMS_FILE 
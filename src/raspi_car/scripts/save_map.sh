#!/bin/bash

# 设置ROS2环境
source ~/raspi_car_ws/install/setup.bash

# 获取当前日期和时间作为地图名称后缀
DATE=$(date +"%Y-%m-%d_%H-%M-%S")
MAP_NAME="raspi_car_map_$DATE"

# 如果提供了参数，使用参数作为地图名称
if [ $# -gt 0 ]; then
  MAP_NAME=$1
fi

echo "保存地图到: $MAP_NAME"

# 保存地图
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: \"$MAP_NAME\"}}"

# 复制到maps目录
MAP_DIR=~/raspi_car_ws/maps
mkdir -p $MAP_DIR
cp $MAP_NAME.* $MAP_DIR/ 2>/dev/null || true

echo "地图已保存到: $MAP_DIR/${MAP_NAME}.[pgm/yaml]" 
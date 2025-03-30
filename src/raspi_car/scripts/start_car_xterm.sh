#!/bin/bash

# 获取参数
DISPLAY_IP=${1:-"192.168.0.4"}

# 设置工作空间
WS_PATH=~/raspi_car_ws
source ${WS_PATH}/install/setup.bash

# 输出信息
echo "启动小车系统，显示IP：${DISPLAY_IP}"

# 确保xterm已安装
if ! command -v xterm &> /dev/null; then
    echo "xterm未安装，尝试安装..."
    sudo apt-get update && sudo apt-get install -y xterm
fi

# 在第一个终端启动基础节点
xterm -title "Car Base" -e "source ${WS_PATH}/install/setup.bash && ros2 launch raspi_car car_base.launch.py; bash" &

# 等待2秒确保基础节点已启动
sleep 2

# 在第二个终端启动激光雷达节点
xterm -title "Lidar" -e "source ${WS_PATH}/install/setup.bash && ros2 launch raspi_car lidar.launch.py; bash" &

# 等待2秒确保激光雷达节点已启动
sleep 2

# 在第三个终端启动RViz
xterm -title "RViz" -e "source ${WS_PATH}/install/setup.bash && ${WS_PATH}/install/raspi_car/share/raspi_car/scripts/start_rviz.sh ${DISPLAY_IP}; bash" &

echo "所有组件已启动，请查看各个终端窗口" 
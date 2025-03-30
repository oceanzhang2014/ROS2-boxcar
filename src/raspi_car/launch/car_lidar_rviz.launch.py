#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
综合启动文件：启动小车基础节点、激光雷达和RViz可视化
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 获取包路径
    pkg_raspi_car = get_package_share_directory('raspi_car')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    display_ip = LaunchConfiguration('display_ip', default='192.168.0.4')
    
    # 设置RVIZ配置文件路径
    rviz_config_file = os.path.join(pkg_raspi_car, 'config', 'car_lidar_view.rviz')
    
    # 包含car_base.launch.py
    car_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_raspi_car, 'launch', 'car_base.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 激光雷达节点
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'serial_port': serial_port,
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'angle_compensate': True
        }]
    )
    
    # 使用直接命令启动RViz
    rviz_cmd = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'export DISPLAY=192.168.0.4:0.0 && export LIBGL_ALWAYS_SOFTWARE=1 && export MESA_GL_VERSION_OVERRIDE=3.3 && ros2 run rviz2 rviz2 -d ' + rviz_config_file
        ],
        output='screen'
    )
    
    # 设置显示环境变量 - 用于其他节点但不用于RViz
    set_display_env = SetEnvironmentVariable(
        name='DISPLAY',
        value=[display_ip, ':0.0']
    )
    
    # 返回启动描述
    return LaunchDescription([
        # 启动参数声明
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='激光雷达串口设备'
        ),
        DeclareLaunchArgument(
            'display_ip',
            default_value='192.168.0.4',
            description='远程显示IP地址'
        ),
        
        # 设置环境变量
        set_display_env,
        
        # 包含小车基础启动文件
        car_base_launch,
        
        # 激光雷达节点
        rplidar_node,
        
        # 使用直接命令启动RViz
        rviz_cmd
    ]) 
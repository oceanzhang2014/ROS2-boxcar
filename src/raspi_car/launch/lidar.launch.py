#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
激光雷达启动文件
启动RPLidar激光雷达节点
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """生成启动描述"""
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    
    # 创建激光雷达节点
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
    
    # 返回启动描述
    return LaunchDescription([
        # 启动参数
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
        
        # 节点
        rplidar_node,
    ]) 
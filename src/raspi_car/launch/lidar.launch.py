#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
激光雷达启动文件
启动RPLidar激光雷达节点，使用最小化基础配置
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
    laser_frame = LaunchConfiguration('laser_frame', default='laser')
    stable_frame = LaunchConfiguration('stable_frame', default='laser_stable')
    use_stable_frame = LaunchConfiguration('use_stable_frame', default='true')
    
    # 决定使用哪个坐标系作为激光雷达帧
    # 当use_stable_frame为true时使用稳定帧，否则使用普通激光雷达帧
    # 使用条件表达式构建最终的frame_id
    frame_id_param = {
        'frame_id': stable_frame,  # 默认使用稳定帧确保数据点方向稳定
    }
    
    # 创建激光雷达节点 - 使用C1官方启动文件的完整配置
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': serial_port,
            'serial_baudrate': 460800,
            'frame_id': stable_frame,
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard'
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
        DeclareLaunchArgument(
            'laser_frame',
            default_value='laser',
            description='激光雷达坐标系名称'
        ),
        DeclareLaunchArgument(
            'stable_frame',
            default_value='laser_stable',
            description='激光雷达稳定坐标系名称'
        ),
        DeclareLaunchArgument(
            'use_stable_frame',
            default_value='true',
            description='是否使用稳定帧作为激光雷达帧'
        ),
        
        # 节点
        rplidar_node,
    ]) 
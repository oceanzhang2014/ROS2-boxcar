#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
激光雷达启动文件
支持不同类型的激光雷达，通过参数指定
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    rplidar_ros_dir = get_package_share_directory('rplidar_ros')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/rplidar')
    baud_rate = LaunchConfiguration('baud_rate', default='460800')
    lidar_type = LaunchConfiguration('lidar_type', default='rplidar')
    channel_type = LaunchConfiguration('channel_type', default='serial')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')
    
    # 声明参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )
    
    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/rplidar',
        description='激光雷达串口设备'
    )
    
    declare_baud_rate_cmd = DeclareLaunchArgument(
        'baud_rate',
        default_value='460800',
        description='激光雷达串口波特率'
    )
    
    declare_lidar_type_cmd = DeclareLaunchArgument(
        'lidar_type',
        default_value='rplidar',
        description='激光雷达类型'
    )
    
    declare_channel_type_cmd = DeclareLaunchArgument(
        'channel_type',
        default_value='serial',
        description='激光雷达通道类型'
    )
    
    declare_scan_mode_cmd = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='激光雷达扫描模式'
    )
    
    # 使用rplidar_ros中的C1激光雷达节点
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'channel_type': channel_type,
            'serial_port': serial_port,
            'serial_baudrate': baud_rate,
            'frame_id': 'laser',
            'angle_compensate': True,
            'scan_mode': scan_mode
        }]
    )
    
    # 返回启动描述
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_serial_port_cmd,
        declare_baud_rate_cmd,
        declare_lidar_type_cmd,
        declare_channel_type_cmd,
        declare_scan_mode_cmd,
        rplidar_node
    ]) 
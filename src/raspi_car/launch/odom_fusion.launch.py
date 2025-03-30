#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
里程计和IMU融合启动文件
启动编码器里程计和IMU-Odom融合节点
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    pkg_raspi_car = get_package_share_directory('raspi_car')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 包含car_base.launch.py (基础节点)
    car_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_raspi_car, 'launch', 'car_base.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 创建编码器里程计节点
    encoder_odom_node = Node(
        package='raspi_car',
        executable='encoder_odom_node.py',
        name='encoder_odom',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'wheel_radius': 0.03},           # 轮子半径 (m)
            {'wheel_separation': 0.16},       # 两轮间距 (m)
            {'encoder_resolution': 20.0},     # 编码器一圈的脉冲数
            {'gear_ratio': 120.0},            # 电机减速比
            {'base_frame': 'base_link'},
            {'odom_frame': 'odom'},
            {'publish_rate': 20.0},           # Hz
        ]
    )
    
    # 创建IMU-Odom融合节点
    imu_odom_fusion_node = Node(
        package='raspi_car',
        executable='imu_odom_fusion.py',
        name='imu_odom_fusion',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'imu_topic': '/imu/data_raw'},
            {'odom_topic': '/odom'},
            {'fused_imu_topic': '/imu/data'},
            {'base_frame': 'base_link'},
            {'odom_frame': 'odom'},
            {'imu_frame': 'imu_link'},
            {'publish_rate': 50.0},          # Hz
            {'alpha': 0.9},                  # 互补滤波器参数，0到1之间
        ]
    )
    
    # 返回启动描述
    return LaunchDescription([
        # 启动参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'
        ),
        
        # 包含基础启动文件
        car_base_launch,
        
        # 节点
        encoder_odom_node,
        imu_odom_fusion_node,
    ]) 
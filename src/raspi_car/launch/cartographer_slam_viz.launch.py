#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Cartographer SLAM启动文件 + 可视化修复节点
包含TF链修复，保证RViz显示正常
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    pkg_raspi_car = get_package_share_directory('raspi_car')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 包含Cartographer SLAM启动文件
    cartographer_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_raspi_car, 'launch', 'cartographer_slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 创建TF链修复节点
    tf_fix_node = Node(
        package='raspi_car',
        executable='fix_tf_chain.py',
        name='tf_fix_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'frame_rate': 30.0},          # 发布频率，每秒30次
            {'map_frame': 'map'},          # 地图坐标系
            {'odom_frame': 'odom'}         # 里程计坐标系
        ]
    )
    
    # 日志信息
    log_start = LogInfo(msg='启动Cartographer SLAM + TF修复节点...')
    log_info = LogInfo(msg='TF修复节点将确保RViz中正确显示机器人和地图')
    
    # 返回启动描述
    return LaunchDescription([
        # 日志信息
        log_start,
        log_info,
        
        # 启动参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'
        ),
        
        # 包含Cartographer SLAM启动文件
        cartographer_slam_launch,
        
        # TF链修复节点
        tf_fix_node,
    ]) 
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
小车基础节点启动文件
启动电机控制器、IMU、TF发布器等基础节点
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    pkg_raspi_car = get_package_share_directory('raspi_car')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 创建TF发布节点
    tf_publisher_node = Node(
        package='raspi_car',
        executable='tf_publisher_node.py',
        name='tf_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_base_frame': 'base_link'},
            {'laser_frame': 'laser'},
            {'laser_position_x': 0.0},
            {'laser_position_y': 0.0},
            {'laser_position_z': 0.1},
        ]
    )
    
    # 创建MPU6050节点
    mpu6050_node = Node(
        package='raspi_car',
        executable='mpu6050_node.py',
        name='mpu6050',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'frame_id': 'imu_link'},
            {'bus_num': 1},
            {'device_addr': 0x68},
            {'publish_rate': 50.0},
        ]
    )
    
    # 创建电机控制节点
    motor_controller_node = Node(
        package='raspi_car',
        executable='motor_controller_node.py',
        name='motor_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}
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
        
        # 节点
        tf_publisher_node,
        mpu6050_node,
        motor_controller_node,
    ]) 
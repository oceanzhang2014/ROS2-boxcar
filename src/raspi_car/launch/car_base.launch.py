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
    publish_odom_tf = LaunchConfiguration('publish_odom_tf', default='true')
    
    # 车轮URDF模型
    urdf_file = os.path.join(pkg_raspi_car, 'urdf', 'raspi_car.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
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
            {'publish_odom_tf': publish_odom_tf},
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
    
    # 创建关节状态发布节点
    joint_state_publisher_node = Node(
        package='raspi_car',
        executable='joint_state_publisher_node.py',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'wheel_radius': 0.03},
            {'wheel_separation': 0.16},
            {'publish_rate': 20.0},
        ]
    )
    
    # 创建编码器里程计节点
    encoder_odom_node = Node(
        package='raspi_car',
        executable='encoder_odom_node.py',
        name='encoder_odom_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'wheel_radius': 0.03},
            {'wheel_separation': 0.16},
            {'encoder_resolution': 20.0},
            {'gear_ratio': 120.0},
            {'base_frame': 'base_link'},
            {'odom_frame': 'odom'},
            {'publish_rate': 20.0},
        ]
    )
    
    # 创建IMU与里程计融合节点
    imu_odom_fusion_node = Node(
        package='raspi_car',
        executable='imu_odom_fusion.py',
        name='imu_odom_fusion',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'imu_frame': 'imu_link'},
            {'base_frame': 'base_link'},
            {'odom_frame': 'odom'},
            {'imu_topic': '/imu/data_raw'},
            {'odom_topic': '/odom'},
            {'publish_rate': 50.0},
            {'alpha': 0.8},
        ]
    )
    
    # 创建机器人状态发布节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_desc},
            {'publish_frequency': 50.0},
            {'frame_prefix': ''},
            {'ignore_timestamp': True},
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
        DeclareLaunchArgument(
            'publish_odom_tf',
            default_value='true',
            description='是否在tf_publisher中发布odom->base_link变换'
        ),
        
        # 节点
        tf_publisher_node,
        mpu6050_node,
        motor_controller_node,
        joint_state_publisher_node,
        encoder_odom_node,
        imu_odom_fusion_node,
        robot_state_publisher_node,
    ]) 
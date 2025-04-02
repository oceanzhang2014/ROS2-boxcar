#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
使用Cartographer进行SLAM建图，与纯IMU定位节点集成
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    pkg_raspi_car = get_package_share_directory('raspi_car')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 设置Cartographer配置文件路径
    cartographer_config_dir = os.path.join(pkg_raspi_car, 'config')
    cartographer_config = os.path.join(cartographer_config_dir, 'raspi_car_cartographer.lua')
    
    # 日志信息
    log_start = LogInfo(msg='启动Cartographer SLAM建图...')
    log_config = LogInfo(msg='使用配置文件: ' + cartographer_config)
    
    # 包含激光雷达启动文件 - 指定使用稳定激光雷达坐标系
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_raspi_car, 'launch', 'lidar.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_stable_frame': 'true',
            'stable_frame': 'laser_stable'
        }.items()
    )
    
    # 创建MPU6050节点 (只保留一个)
    mpu6050_node = Node(
        package='raspi_car',
        executable='mpu6050_node.py',
        name='mpu6050',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'frame_id': 'imu_link'},
            {'bus_num': 11},
            {'device_addr': 0x68},
            {'publish_rate': 100.0},  # Hz - 提高采样率以获得更好的效果
            {'ignore_connection_errors': True},  # 添加此参数，忽略连接错误
        ]
    )
    
    # 创建基于IMU的里程计节点
    imu_odometry_node = Node(
        package='raspi_car',
        executable='imu_odom_fusion.py',
        name='imu_based_odometry',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'imu_topic': '/imu/data_raw'},
            {'base_frame': 'base_link'},
            {'odom_frame': 'odom'},
            {'world_frame': 'map'},
            {'imu_frame': 'imu_link'},
            {'publish_rate': 30.0},                # 降低到30Hz，减少TF树更新频率
            {'madgwick_beta': 0.01},               # 显著降低Madgwick滤波器增益，提高稳定性
            {'gravity_magnitude': 9.81},           # 重力加速度
            {'imu_acceleration_bias': 0.0},        # IMU加速度偏差
            {'imu_angular_velocity_bias': 0.0},    # IMU角速度偏差
            {'use_mag': False},                    # 不使用磁力计
            {'rotation_stabilizer': 0.995},        # 极大增加旋转稳定系数，提高稳定性
            {'zero_yaw_period': 10.0},             # 大幅增加零偏航角修正周期，减少频繁调整
        ]
    )
    
    # TF发布器节点，更新参数以稳定激光雷达方向
    tf_publisher_node = Node(
        package='raspi_car',
        executable='tf_publisher_node.py',
        name='tf_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_base_frame': 'base_link'},
            {'laser_frame': 'laser'},
            {'imu_frame': 'imu_link'},
            {'wheel_frames': True},               # 发布轮胎坐标系
            {'laser_static_orientation': True},   # 确保激光雷达方向保持静态
            {'laser_stable_frame': 'laser_stable'}, # 指定稳定激光雷达坐标系名称
        ]
    )
    
    # 添加自定义关节状态发布器
    joint_state_pub_cmd = Node(
        package='raspi_car',
        executable='joint_state_pub.py',
        name='simple_joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'publish_rate': 1.0},  # 极低的发布频率，减少TF树不必要的更新
            {'use_fixed_positions': True},  # 使用固定位置以确保稳定性
        ]
    )
    
    # 机器人状态发布器节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': open(os.path.join(pkg_raspi_car, 'urdf', 'raspi_car.urdf'), 'r').read()}
        ]
    )
    
    # 创建Cartographer节点 - 调整参数以解决坐标系跳动问题
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', 'raspi_car_cartographer.lua'],
        remappings=[
            ('scan', '/scan'),
            # 修改：不再需要IMU数据，完全基于激光雷达进行SLAM
            # ('odom', '/odometry/imu_only')  # 注释掉IMU里程计
        ]
    )
    
    # 创建Cartographer占用栅格地图发布节点
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': 0.05,           # 地图分辨率 
            'publish_period_sec': 1.0     # 降低发布周期至1秒，使地图更频繁更新
        }]
    )
    
    # 返回启动描述，确保正确的启动顺序
    return LaunchDescription([
        # 日志信息
        log_start,
        log_config,
        
        # 启动参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间'
        ),
        
        # 按照依赖顺序启动 - 优化启动顺序以确保稳定的TF树构建
        robot_state_publisher,   # 首先发布机器人模型
        joint_state_pub_cmd,     # 关节状态发布
        tf_publisher_node,       # 发布静态TF树
        
        # 添加2.0秒延迟，让上述节点有足够时间初始化
        TimerAction(
            period=2.0,
            actions=[
                mpu6050_node,            # IMU传感器
            ]
        ),
        
        # 添加3.0秒延迟，让IMU传感器有足够时间初始化
        TimerAction(
            period=3.0,
            actions=[
                imu_odometry_node,     # IMU里程计
            ]
        ),
        
        # 添加4.0秒延迟，等待IMU里程计初始化完成
        TimerAction(
            period=4.0,
            actions=[
                lidar_launch,          # 激光雷达
            ]
        ),
        
        # 添加6.0秒延迟，确保传感器正常运行后再启动Cartographer
        TimerAction(
            period=6.0,
            actions=[
                # Cartographer节点 - 最后启动
                cartographer_node,
                cartographer_occupancy_grid_node,
            ]
        ),
    ]) 
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
使用Cartographer进行SLAM建图
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    pkg_raspi_car = get_package_share_directory('raspi_car')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf', default='true')
    display_ip = LaunchConfiguration('display_ip', default='192.168.0.4')
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    
    # 设置Cartographer配置文件路径
    cartographer_config_dir = os.path.join(pkg_raspi_car, 'config')
    cartographer_config = os.path.join(cartographer_config_dir, 'raspi_car_cartographer.lua')
    
    # 设置RViz配置文件路径
    rviz_config_file = os.path.join(pkg_raspi_car, 'config', 'cartographer.rviz')
    
    # 包含基础启动文件
    car_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_raspi_car, 'launch', 'car_base.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'publish_odom_tf': publish_odom_tf
        }.items()
    )
    
    # 包含激光雷达启动文件
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_raspi_car, 'launch', 'lidar.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 创建Cartographer节点
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
            ('imu', '/imu/data'),   # 使用融合后的IMU数据
            ('odom', '/odom')       # 使用里程计数据
        ]
    )
    
    # 创建Cartographer占用栅格地图发布节点
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'resolution': 0.05}]
    )
    
    # 使用直接命令启动RViz
    display_ip_str = TextSubstitution(text="192.168.0.4")
    rviz_cmd = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'export DISPLAY={display_ip_str}:0.0 && export LIBGL_ALWAYS_SOFTWARE=1 && export MESA_GL_VERSION_OVERRIDE=3.3 && export XDG_RUNTIME_DIR=/run/user/$(id -u) && ros2 run rviz2 rviz2 -d {rviz_config_file}'
        ],
        output='screen',
        condition=IfCondition(launch_rviz)
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
        DeclareLaunchArgument(
            'display_ip',
            default_value='192.168.0.4',
            description='远程显示IP地址'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='是否启动RViz'
        ),
        
        # 包含的启动文件
        car_base_launch,
        lidar_launch,
        
        # Cartographer节点
        cartographer_node,
        cartographer_occupancy_grid_node,
        
        # 使用直接命令启动RViz（条件启动）
        rviz_cmd,
    ]) 
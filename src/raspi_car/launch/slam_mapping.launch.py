#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
SLAM建图启动文件
使用slam_toolbox进行在线异步SLAM建图
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
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
    
    # 设置SLAM参数文件
    slam_params_file = os.path.join(pkg_raspi_car, 'config', 'slam_params.yaml')
    
    # 设置RViz配置文件路径
    rviz_config_file = os.path.join(pkg_raspi_car, 'config', 'slam_mapping.rviz')
    
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
    
    # 创建SLAM Toolbox节点
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 避免直接在launch文件中使用LaunchConfiguration对象，而是创建固定的字符串表示
    display_ip_str = TextSubstitution(text="192.168.0.4")
    
    # 使用直接命令启动RViz
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
        
        # SLAM节点
        slam_toolbox_node,
        
        # 使用直接命令启动RViz（条件启动）
        rviz_cmd,
    ]) 
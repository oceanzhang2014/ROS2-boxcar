#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
小车完整启动文件
启动所有节点，包括基础节点和RViz可视化
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    pkg_raspi_car = get_package_share_directory('raspi_car')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf', default='false')
    display_ip = LaunchConfiguration('display_ip', default='192.168.0.4')
    
    # 设置RViz配置文件路径
    rviz_config_file = os.path.join(pkg_raspi_car, 'config', 'car_lidar_view.rviz')
    
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
    
    # 设置显示环境变量
    set_display_env = SetEnvironmentVariable(
        name='DISPLAY',
        value=[display_ip, ':0.0']
    )
    
    # 设置OpenGL软件渲染
    set_libgl_env = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='1'
    )
    
    # 设置Mesa GL版本
    set_mesa_gl_env = SetEnvironmentVariable(
        name='MESA_GL_VERSION_OVERRIDE',
        value='3.3'
    )
    
    # 使用直接命令启动RViz
    rviz_cmd = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'export DISPLAY={display_ip}:0.0 && export LIBGL_ALWAYS_SOFTWARE=1 && export MESA_GL_VERSION_OVERRIDE=3.3 && export XDG_RUNTIME_DIR=/run/user/$(id -u) && ros2 run rviz2 rviz2 -d {rviz_config_file}'
        ],
        output='screen'
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
            default_value='false',
            description='是否在tf_publisher中发布odom->base_link变换'
        ),
        DeclareLaunchArgument(
            'display_ip',
            default_value='192.168.0.4',
            description='远程显示IP地址'
        ),
        
        # 环境变量设置
        set_display_env,
        set_libgl_env,
        set_mesa_gl_env,
        
        # 包含的启动文件
        car_base_launch,
        lidar_launch,
        
        # 使用直接命令启动RViz
        rviz_cmd,
    ]) 
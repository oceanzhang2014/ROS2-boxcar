#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
小车RViz启动文件
启动RViz以可视化小车状态
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """生成启动描述"""
    
    # 获取包路径
    pkg_raspi_car = get_package_share_directory('raspi_car')
    
    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config = LaunchConfiguration('rviz_config', default=os.path.join(
        pkg_raspi_car, 'config', 'car_lidar_view.rviz'))
    display_ip = LaunchConfiguration('display_ip', default='192.168.0.4')
    
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
    
    # 创建RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config]
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
            'rviz_config',
            default_value=os.path.join(pkg_raspi_car, 'config', 'car_lidar_view.rviz'),
            description='RViz配置文件路径'
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
        
        # 节点
        rviz_node,
    ]) 
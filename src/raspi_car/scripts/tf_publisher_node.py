#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TF发布节点: 只发布机器人的静态坐标系统

此节点仅维护机器人的静态坐标关系，例如base_link到laser或imu_link的固定变换
动态变换完全由IMU里程计和Cartographer负责
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import tf_transformations
import time
import math

class TFPublisherNode(Node):
    """TF发布节点 - 仅静态变换"""

    def __init__(self):
        super().__init__('car_tf_publisher')
        
        # 声明参数
        self.declare_parameter_if_not_declared('robot_base_frame', 'base_link')
        self.declare_parameter_if_not_declared('laser_frame', 'laser')
        self.declare_parameter_if_not_declared('imu_frame', 'imu_link')
        
        # 激光雷达相对于机器人底盘的位置和朝向
        self.declare_parameter_if_not_declared('laser_x', 0.0)  # 前后偏移，前为正
        self.declare_parameter_if_not_declared('laser_y', 0.0)  # 左右偏移，左为正
        self.declare_parameter_if_not_declared('laser_z', 0.1)  # 高度偏移
        self.declare_parameter_if_not_declared('laser_roll', 0.0)  # 横滚角
        self.declare_parameter_if_not_declared('laser_pitch', 0.0)  # 俯仰角
        self.declare_parameter_if_not_declared('laser_yaw', 0.0)  # 偏航角
        
        # IMU相对于机器人底盘的位置和朝向
        self.declare_parameter_if_not_declared('imu_x', 0.0)
        self.declare_parameter_if_not_declared('imu_y', 0.0)
        self.declare_parameter_if_not_declared('imu_z', 0.05)
        self.declare_parameter_if_not_declared('imu_roll', 0.0)
        self.declare_parameter_if_not_declared('imu_pitch', 0.0)
        self.declare_parameter_if_not_declared('imu_yaw', 0.0)
        
        # 轮胎相对位置 - 添加轮胎坐标系的支持
        self.declare_parameter_if_not_declared('wheel_frames', True)  # 是否发布轮胎坐标系
        
        # 激光雷达方向固定参数
        self.declare_parameter_if_not_declared('laser_static_orientation', True)  # 是否固定激光雷达方向
        self.declare_parameter_if_not_declared('laser_stable_frame', 'laser_stable')  # 固定方向框架名称
        
        # 获取参数
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        
        self.laser_x = self.get_parameter('laser_x').value
        self.laser_y = self.get_parameter('laser_y').value
        self.laser_z = self.get_parameter('laser_z').value
        self.laser_roll = self.get_parameter('laser_roll').value
        self.laser_pitch = self.get_parameter('laser_pitch').value
        self.laser_yaw = self.get_parameter('laser_yaw').value
        
        self.imu_x = self.get_parameter('imu_x').value
        self.imu_y = self.get_parameter('imu_y').value
        self.imu_z = self.get_parameter('imu_z').value
        self.imu_roll = self.get_parameter('imu_roll').value
        self.imu_pitch = self.get_parameter('imu_pitch').value
        self.imu_yaw = self.get_parameter('imu_yaw').value
        
        self.wheel_frames = self.get_parameter('wheel_frames').value
        self.laser_static_orientation = self.get_parameter('laser_static_orientation').value
        self.laser_stable_frame = self.get_parameter('laser_stable_frame').value
        
        # 创建静态变换广播器
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 创建动态变换广播器（用于激光雷达稳定化）
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 发布静态变换
        self.publish_static_transforms()
        
        # 创建一个低频率定时器定期重新发布静态变换（确保TF树完整）
        self.timer = self.create_timer(5.0, self.refresh_static_transforms)
        
        # 如果启用了激光雷达方向稳定，直接让laser_stable在静态发布器中处理
        if self.laser_static_orientation:
            self.get_logger().info(f'已启用激光雷达完全等效坐标系: {self.laser_stable_frame}')
        
        self.get_logger().info('TF发布节点已初始化 - 仅发布静态变换')
        self.get_logger().info(f'机器人坐标系: {self.robot_base_frame}')
        self.get_logger().info(f'激光雷达坐标系: {self.laser_frame}，位置: [{self.laser_x}, {self.laser_y}, {self.laser_z}]')
        self.get_logger().info(f'IMU坐标系: {self.imu_frame}，位置: [{self.imu_x}, {self.imu_y}, {self.imu_z}]')
        if self.wheel_frames:
            self.get_logger().info('将发布轮胎坐标系')
    
    def declare_parameter_if_not_declared(self, name, value):
        """安全地声明参数，避免重复声明错误"""
        try:
            return self.declare_parameter(name, value)
        except:
            return self.get_parameter(name)
    
    def refresh_static_transforms(self):
        """定期重新发布静态变换，确保TF树完整"""
        self.publish_static_transforms()
        self.get_logger().debug('刷新静态TF变换')
    
    def publish_static_transforms(self):
        """发布静态变换"""
        try:
            transforms = []
            
            # 发布 base_link -> laser 变换
            laser_transform = TransformStamped()
            laser_transform.header.stamp = self.get_clock().now().to_msg()
            laser_transform.header.frame_id = self.robot_base_frame
            laser_transform.child_frame_id = self.laser_frame
            
            # 设置激光雷达相对于底盘的位置
            laser_transform.transform.translation.x = self.laser_x
            laser_transform.transform.translation.y = self.laser_y
            laser_transform.transform.translation.z = self.laser_z
            
            # 设置激光雷达相对于底盘的朝向
            q = tf_transformations.quaternion_from_euler(
                self.laser_roll, self.laser_pitch, self.laser_yaw)
            laser_transform.transform.rotation.x = q[0]
            laser_transform.transform.rotation.y = q[1]
            laser_transform.transform.rotation.z = q[2]
            laser_transform.transform.rotation.w = q[3]
            
            transforms.append(laser_transform)
            
            # 如果启用了激光雷达稳定帧，直接从base_link创建到laser_stable的变换
            # 这样确保laser_stable与laser的位置和方向完全相同
            if self.laser_static_orientation:
                stable_transform = TransformStamped()
                stable_transform.header.stamp = self.get_clock().now().to_msg()
                stable_transform.header.frame_id = self.robot_base_frame
                stable_transform.child_frame_id = self.laser_stable_frame
                
                # 设置与laser完全相同的位置
                stable_transform.transform.translation.x = self.laser_x
                stable_transform.transform.translation.y = self.laser_y
                stable_transform.transform.translation.z = self.laser_z
                
                # 设置与laser完全相同的方向
                stable_transform.transform.rotation.x = q[0]
                stable_transform.transform.rotation.y = q[1]
                stable_transform.transform.rotation.z = q[2]
                stable_transform.transform.rotation.w = q[3]
                
                transforms.append(stable_transform)
            
            # 发布 base_link -> imu_link 变换
            imu_transform = TransformStamped()
            imu_transform.header.stamp = self.get_clock().now().to_msg()
            imu_transform.header.frame_id = self.robot_base_frame
            imu_transform.child_frame_id = self.imu_frame
            
            # 设置IMU相对于底盘的位置
            imu_transform.transform.translation.x = self.imu_x
            imu_transform.transform.translation.y = self.imu_y
            imu_transform.transform.translation.z = self.imu_z
            
            # 设置IMU相对于底盘的朝向
            q = tf_transformations.quaternion_from_euler(
                self.imu_roll, self.imu_pitch, self.imu_yaw)
            imu_transform.transform.rotation.x = q[0]
            imu_transform.transform.rotation.y = q[1]
            imu_transform.transform.rotation.z = q[2]
            imu_transform.transform.rotation.w = q[3]
            
            transforms.append(imu_transform)
            
            # 发布轮胎坐标系
            if self.wheel_frames:
                # 定义轮胎的位置 - 这些值应与URDF中的定义匹配
                wheel_positions = {
                    'front_left_wheel': [0.08, 0.08, -0.025, -1.5707, 0, 0],
                    'front_right_wheel': [0.08, -0.08, -0.025, -1.5707, 0, 0],
                    'rear_left_wheel': [-0.08, 0.08, -0.025, -1.5707, 0, 0],
                    'rear_right_wheel': [-0.08, -0.08, -0.025, -1.5707, 0, 0]
                }
                
                # 为每个轮胎创建静态变换
                for wheel_name, position in wheel_positions.items():
                    wheel_transform = TransformStamped()
                    wheel_transform.header.stamp = self.get_clock().now().to_msg()
                    wheel_transform.header.frame_id = self.robot_base_frame
                    wheel_transform.child_frame_id = wheel_name
                    
                    # 设置轮胎相对于底盘的位置
                    wheel_transform.transform.translation.x = position[0]
                    wheel_transform.transform.translation.y = position[1]
                    wheel_transform.transform.translation.z = position[2]
                    
                    # 设置轮胎相对于底盘的朝向
                    q = tf_transformations.quaternion_from_euler(
                        position[3], position[4], position[5])
                    wheel_transform.transform.rotation.x = q[0]
                    wheel_transform.transform.rotation.y = q[1]
                    wheel_transform.transform.rotation.z = q[2]
                    wheel_transform.transform.rotation.w = q[3]
                    
                    transforms.append(wheel_transform)
            
            # 发布所有静态变换
            self.static_tf_broadcaster.sendTransform(transforms)
            
        except Exception as e:
            self.get_logger().error(f'发布静态变换时出错: {str(e)}')

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    tf_publisher = TFPublisherNode()
    
    try:
        rclpy.spin(tf_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        tf_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
关节状态发布节点: 发布车轮关节状态

此节点根据cmd_vel订阅的速度命令，模拟计算车轮旋转角度和速度，并发布关节状态
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math
import time

class JointStatePublisherNode(Node):
    """关节状态发布节点"""
    
    def __init__(self):
        super().__init__('joint_state_publisher')
        
        # 声明参数 - 使用安全的方法声明
        self.declare_parameter_if_not_declared('use_sim_time', False)
        self.declare_parameter_if_not_declared('wheel_radius', 0.03)  # 轮子半径 (m)
        self.declare_parameter_if_not_declared('wheel_separation', 0.16)  # 两轮间距 (m)
        self.declare_parameter_if_not_declared('publish_rate', 20.0)  # Hz
        
        # 获取参数
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # 创建关节状态发布者
        self.joint_state_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10)
        
        # 订阅速度命令
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # 车轮关节名称 - 必须与URDF中完全一致
        self.wheel_joints = [
            'base_to_front_left_wheel',   # 左前轮关节
            'base_to_front_right_wheel',  # 右前轮关节
            'base_to_rear_left_wheel',    # 左后轮关节
            'base_to_rear_right_wheel',   # 右后轮关节
        ]
        
        # 车轮状态
        self.wheel_positions = {joint: 0.0 for joint in self.wheel_joints}
        self.wheel_velocities = {joint: 0.0 for joint in self.wheel_joints}
        
        # 当前速度命令
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # 上次更新时间
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        
        # 计数器，用于日志打印
        self.count = 0
        
        # 创建定时器发布关节状态
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_states)
        
        self.get_logger().info('关节状态发布节点已启动')
        self.get_logger().info(f'发布频率: {self.publish_rate} Hz')
        self.get_logger().info(f'车轮关节: {self.wheel_joints}')
    
    def cmd_vel_callback(self, msg):
        """速度命令回调处理"""
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
    
    def publish_joint_states(self):
        """发布关节状态"""
        # 计算时间差
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 确保时间差不为零
        if dt <= 0:
            return
        
        # 计算左右轮速度
        left_wheel_vel = (self.linear_x - self.angular_z * self.wheel_separation / 2) / self.wheel_radius
        right_wheel_vel = (self.linear_x + self.angular_z * self.wheel_separation / 2) / self.wheel_radius
        
        # 更新轮子速度
        self.wheel_velocities['base_to_front_left_wheel'] = left_wheel_vel
        self.wheel_velocities['base_to_rear_left_wheel'] = left_wheel_vel
        self.wheel_velocities['base_to_front_right_wheel'] = right_wheel_vel
        self.wheel_velocities['base_to_rear_right_wheel'] = right_wheel_vel
        
        # 更新轮子位置（积分角度）
        for joint in self.wheel_joints:
            self.wheel_positions[joint] += self.wheel_velocities[joint] * dt
            # 限制在 -pi 到 pi 范围内
            self.wheel_positions[joint] = ((self.wheel_positions[joint] + math.pi) % (2 * math.pi)) - math.pi
        
        # 创建并发布关节状态消息
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # 添加关节名称
        joint_state.name = self.wheel_joints
        
        # 添加关节位置
        joint_state.position = [self.wheel_positions[joint] for joint in joint_state.name]
        
        # 添加关节速度
        joint_state.velocity = [self.wheel_velocities[joint] for joint in joint_state.name]
        
        # 发布关节状态
        self.joint_state_publisher.publish(joint_state)
        
        # 更新计数器
        self.count += 1
        if self.count % 100 == 0:  # 每100次发布一次日志
            self.get_logger().debug(f'发布关节状态: {joint_state.name}')

    def declare_parameter_if_not_declared(self, name, value):
        """安全地声明参数，避免重复声明错误"""
        try:
            return self.declare_parameter(name, value)
        except Exception as e:
            self.get_logger().debug(f'参数 {name} 已经被声明或声明失败: {str(e)}')
            return self.get_parameter(name)

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = JointStatePublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
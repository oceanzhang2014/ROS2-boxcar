#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
编码器里程计节点: 基于电机编码器计算机器人里程计

此节点读取电机编码器数据，计算机器人位置、速度，并发布里程计消息
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
import time
import numpy as np
from transforms3d.euler import euler2quat

class EncoderOdomNode(Node):
    """编码器里程计节点"""
    
    def __init__(self):
        super().__init__('encoder_odom_node')
        
        # 声明参数
        self.declare_parameter_if_not_declared('use_sim_time', False)
        self.declare_parameter_if_not_declared('wheel_radius', 0.035)  # 轮子半径 (m)
        self.declare_parameter_if_not_declared('wheel_separation', 0.145)  # 两轮间距 (m)
        self.declare_parameter_if_not_declared('encoder_resolution', 20.0)  # 编码器一圈的脉冲数
        self.declare_parameter_if_not_declared('gear_ratio', 120.0)  # 电机减速比
        self.declare_parameter_if_not_declared('base_frame', 'base_link')
        self.declare_parameter_if_not_declared('odom_frame', 'odom')
        self.declare_parameter_if_not_declared('publish_rate', 20.0)  # Hz
        
        # 获取参数
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.encoder_resolution = self.get_parameter('encoder_resolution').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # 创建发布者
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10)
        
        # 创建订阅者
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 里程计状态
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_left_ticks = 0
        self.last_right_ticks = 0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        
        # 由于我们可能没有实际的编码器，我们也可以使用cmd_vel来模拟里程计
        self.vx = 0.0
        self.vth = 0.0
        self.use_cmd_vel = True
        
        # 创建定时器发布里程计数据
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odom)
        
        self.get_logger().info('编码器里程计节点已启动')
        self.get_logger().info(f'轮子半径: {self.wheel_radius} m, 轮距: {self.wheel_separation} m')
        self.get_logger().info(f'编码器分辨率: {self.encoder_resolution}, 减速比: {self.gear_ratio}')
        self.get_logger().info(f'使用cmd_vel模拟里程计: {self.use_cmd_vel}')
    
    def declare_parameter_if_not_declared(self, name, value):
        """安全地声明参数，避免重复声明错误"""
        try:
            return self.declare_parameter(name, value)
        except Exception as e:
            self.get_logger().debug(f'参数 {name} 已经被声明或声明失败: {str(e)}')
            return self.get_parameter(name)
    
    def joint_state_callback(self, msg):
        """关节状态回调处理"""
        try:
            # 左右轮名称
            left_wheel_joints = ['base_to_front_left_wheel', 'base_to_rear_left_wheel']
            right_wheel_joints = ['base_to_front_right_wheel', 'base_to_rear_right_wheel']
            
            # 计算左侧轮平均角速度
            left_wheel_vel_sum = 0.0
            left_wheel_count = 0
            for joint_name in left_wheel_joints:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    if idx < len(msg.velocity):
                        left_wheel_vel_sum += msg.velocity[idx]
                        left_wheel_count += 1
            
            # 计算右侧轮平均角速度
            right_wheel_vel_sum = 0.0
            right_wheel_count = 0
            for joint_name in right_wheel_joints:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    if idx < len(msg.velocity):
                        right_wheel_vel_sum += msg.velocity[idx]
                        right_wheel_count += 1
            
            # 计算左右轮平均角速度
            if left_wheel_count > 0:
                self.left_wheel_vel = left_wheel_vel_sum / left_wheel_count
            
            if right_wheel_count > 0:
                self.right_wheel_vel = right_wheel_vel_sum / right_wheel_count
            
            # 如果有足够的数据，关闭cmd_vel模拟
            if left_wheel_count > 0 and right_wheel_count > 0:
                self.use_cmd_vel = False
                self.get_logger().debug('使用编码器数据计算里程计')
            
        except Exception as e:
            self.get_logger().warning(f'关节状态回调处理错误: {str(e)}')
    
    def cmd_vel_callback(self, msg):
        """速度命令回调处理"""
        if self.use_cmd_vel:
            # 使用cmd_vel来模拟里程计
            self.vx = msg.linear.x
            self.vth = msg.angular.z
    
    def publish_odom(self):
        """发布里程计数据"""
        # 当前时间
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 确保时间差不为零
        if dt == 0:
            return
        
        # 根据使用模式计算位移和旋转
        if self.use_cmd_vel:
            # 使用cmd_vel命令来更新位置
            delta_x = self.vx * math.cos(self.theta) * dt
            delta_y = self.vx * math.sin(self.theta) * dt
            delta_theta = self.vth * dt
        else:
            # 计算左右轮的线速度
            vl = self.left_wheel_vel * self.wheel_radius
            vr = self.right_wheel_vel * self.wheel_radius
            
            # 计算机器人的线速度和角速度
            vx = (vr + vl) / 2.0
            vth = (vr - vl) / self.wheel_separation
            
            # 计算位移和旋转
            delta_x = vx * math.cos(self.theta) * dt
            delta_y = vx * math.sin(self.theta) * dt
            delta_theta = vth * dt
        
        # 更新位置和朝向
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        
        # 将欧拉角转换为四元数
        q = euler2quat(0, 0, self.theta, 'sxyz')
        
        # 创建并发布odom->base_link变换
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        
        # 发布TF变换
        self.tf_broadcaster.sendTransform(t)
        
        # 创建并发布Odometry消息
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # 设置位置
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        
        # 设置速度
        if self.use_cmd_vel:
            odom.twist.twist.linear.x = self.vx
            odom.twist.twist.angular.z = self.vth
        else:
            vl = self.left_wheel_vel * self.wheel_radius
            vr = self.right_wheel_vel * self.wheel_radius
            odom.twist.twist.linear.x = (vr + vl) / 2.0
            odom.twist.twist.angular.z = (vr - vl) / self.wheel_separation
        
        # 发布里程计
        self.odom_publisher.publish(odom)

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = EncoderOdomNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
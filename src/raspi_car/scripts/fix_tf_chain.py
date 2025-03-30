#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TF链修复节点：提供完整的坐标系统链

此节点提供map->odom->base_link->laser的完整变换链，并且处理URDF模型变换，
确保RViz可以正确显示机器人模型和地图
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
import tf_transformations
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np
from rclpy.time import Time
from rclpy.duration import Duration

class TFChainFixerNode(Node):
    """TF链修复节点"""

    def __init__(self):
        super().__init__('tf_chain_fixer')
        
        # 声明参数
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('publish_rate', 50.0)  # 提高发布频率
        self.declare_parameter('future_buffer', 0.1)  # 添加未来时间缓冲，以秒为单位
        
        # 获取参数
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.future_buffer = self.get_parameter('future_buffer').value
        
        # 创建变换广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 创建TF缓冲和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建关节状态发布器
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # 车轮关节名称
        self.joints = [
            'base_to_front_left_wheel',
            'base_to_front_right_wheel',
            'base_to_rear_left_wheel',
            'base_to_rear_right_wheel'
        ]
        
        # 发布静态变换（只需发布一次）
        self.publish_static_transforms()
        
        # 创建定时器，定期发布变换
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_tf_and_joints)
        
        self.get_logger().info('TF链修复节点已初始化')
        self.get_logger().info(f'将提供完整TF链: {self.map_frame} -> {self.odom_frame} -> {self.robot_base_frame} -> {self.laser_frame}')
        self.get_logger().info(f'将发布关节状态: {", ".join(self.joints)}')
        self.get_logger().info(f'TF发布频率: {self.publish_rate} Hz, 未来时间缓冲: {self.future_buffer} 秒')
    
    def publish_static_transforms(self):
        """发布静态变换（laser到base_link）"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.robot_base_frame
        t.child_frame_id = self.laser_frame
        
        # 设置激光雷达相对于base_link的位置（根据实际情况调整）
        t.transform.translation.x = 0.0  # 假设激光雷达位于基座前方0.1米
        t.transform.translation.y = 0.0  # 假设激光雷达在中心线上
        t.transform.translation.z = 0.05  # 假设激光雷达高于基座0.05米
        
        # 假设激光雷达水平放置，不旋转
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'已发布静态变换: {self.robot_base_frame} -> {self.laser_frame}')
    
    def publish_tf_and_joints(self):
        """发布变换和关节状态"""
        # 获取当前时间并添加未来缓冲
        now = self.get_clock().now()
        future_time = Time.from_msg(now.to_msg())
        future_time = future_time + Duration(seconds=self.future_buffer)
        
        # 发布map->odom变换（带未来时间缓冲）
        self.publish_map_to_odom(future_time)
        
        # 发布odom->base_link变换（带未来时间缓冲）
        self.publish_odom_to_base(future_time)
        
        # 发布关节状态
        self.publish_joint_states(future_time)
    
    def publish_map_to_odom(self, time_stamp):
        """发布map->odom变换"""
        t = TransformStamped()
        t.header.stamp = time_stamp.to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        
        # 默认恒等变换，在实际应用中应由SLAM系统提供
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_odom_to_base(self, time_stamp):
        """发布odom->base_link变换"""
        t = TransformStamped()
        t.header.stamp = time_stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.robot_base_frame
        
        # 默认恒等变换，在实际应用中应由里程计数据更新
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_joint_states(self, time_stamp):
        """发布关节状态"""
        # 创建关节状态消息
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = time_stamp.to_msg()
        joint_state.name = self.joints
        
        # 设置关节位置（角度）
        # 这里可以设置实际的轮子旋转角度，先用0代替
        joint_state.position = [0.0, 0.0, 0.0, 0.0]
        
        # 发布关节状态
        self.joint_state_publisher.publish(joint_state)

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    tf_chain_fixer = TFChainFixerNode()
    
    try:
        rclpy.spin(tf_chain_fixer)
    except KeyboardInterrupt:
        pass
    finally:
        tf_chain_fixer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
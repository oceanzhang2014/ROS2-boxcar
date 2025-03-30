#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TF发布节点: 发布机器人坐标系统

此节点维护机器人的坐标系统，发布不同部分之间的坐标变换
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from tf2_ros import StaticTransformBroadcaster
import tf_transformations

class TFPublisherNode(Node):
    """TF发布节点"""

    def __init__(self):
        super().__init__('tf_publisher')
        
        # 声明参数
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser')
        
        # 激光雷达相对于机器人底盘的位置和朝向
        self.declare_parameter('laser_x', 0.0)  # 前后偏移，前为正
        self.declare_parameter('laser_y', 0.0)  # 左右偏移，左为正
        self.declare_parameter('laser_z', 0.1)  # 高度偏移
        self.declare_parameter('laser_roll', 0.0)  # 横滚角
        self.declare_parameter('laser_pitch', 0.0)  # 俯仰角
        self.declare_parameter('laser_yaw', 0.0)  # 偏航角
        
        # 获取参数
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.laser_frame = self.get_parameter('laser_frame').value
        
        self.laser_x = self.get_parameter('laser_x').value
        self.laser_y = self.get_parameter('laser_y').value
        self.laser_z = self.get_parameter('laser_z').value
        self.laser_roll = self.get_parameter('laser_roll').value
        self.laser_pitch = self.get_parameter('laser_pitch').value
        self.laser_yaw = self.get_parameter('laser_yaw').value
        
        # 创建变换广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 发布静态变换 (base_link -> laser)
        self.publish_static_transforms()
        
        # 创建定时器，定期发布变换
        self.timer = self.create_timer(0.1, self.publish_tf)
        
        self.get_logger().info('TF发布节点已初始化')
        self.get_logger().info(f'机器人坐标系: {self.robot_base_frame}')
        self.get_logger().info(f'激光雷达坐标系: {self.laser_frame}，位置: [{self.laser_x}, {self.laser_y}, {self.laser_z}]')
    
    def publish_static_transforms(self):
        """发布静态变换"""
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
        
        # 发布静态变换
        self.static_tf_broadcaster.sendTransform(laser_transform)
        
        self.get_logger().info('已发布静态变换: base_link -> laser')
    
    def publish_tf(self):
        """发布动态变换"""
        # 在SLAM建图过程中，map->odom变换由SLAM工具包生成
        # 此处仅发布odom->base_link变换
        
        # 例如：发布odom->base_link的恒等变换
        # 注意：在实际应用中，这个变换应该根据里程计数据动态更新
        odom_transform = TransformStamped()
        odom_transform.header.stamp = self.get_clock().now().to_msg()
        odom_transform.header.frame_id = self.odom_frame
        odom_transform.child_frame_id = self.robot_base_frame
        
        # 设置位置（恒等变换）
        odom_transform.transform.translation.x = 0.0
        odom_transform.transform.translation.y = 0.0
        odom_transform.transform.translation.z = 0.0
        
        # 设置方向（恒等变换）
        odom_transform.transform.rotation.x = 0.0
        odom_transform.transform.rotation.y = 0.0
        odom_transform.transform.rotation.z = 0.0
        odom_transform.transform.rotation.w = 1.0
        
        # 发布变换
        self.tf_broadcaster.sendTransform(odom_transform)

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
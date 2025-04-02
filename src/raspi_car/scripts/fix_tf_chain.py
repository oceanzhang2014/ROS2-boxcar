#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
TF链修复节点
当Cartographer未发布 map->odom 变换时，提供所需的变换
仅用于调试、可视化目的
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf2_ros
import math
import sys

class TFFixNode(Node):
    """TF链修复节点，提供缺失的map->odom变换"""

    def __init__(self):
        super().__init__('tf_fix_node')
        
        # 创建参数
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')
        
        # 获取参数
        self.frame_rate = self.get_parameter('frame_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 创建TF缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 创建定时器
        self.timer = self.create_timer(1.0 / self.frame_rate, self.publish_tf)
        
        # 日志信息
        self.get_logger().info('TF链修复节点已启动')
        self.get_logger().info(f'将为 {self.map_frame} -> {self.odom_frame} 发布变换')
        self.get_logger().info(f'发布频率: {self.frame_rate} Hz')
        
        # 初始化计数器
        self.counter = 0
        self.warning_interval = 100  # 每100次迭代输出一次警告，避免刷屏
    
    def publish_tf(self):
        """发布静态TF变换: map -> odom"""
        
        # 检查是否真的缺少map->odom的变换
        try:
            # 尝试获取map->odom变换
            self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=0.1)
            )
            
            # 如果成功获取，则Cartographer已经在发布，我们不需要发布
            if self.counter % self.warning_interval == 0:
                self.get_logger().info('Cartographer已在发布map->odom变换，无需额外发布')
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # 如果没有找到变换，我们需要发布
            t = TransformStamped()
            
            # 设置头信息
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.map_frame
            
            # 设置子框架
            t.child_frame_id = self.odom_frame
            
            # 设置变换 - 默认为恒等变换 (没有旋转和平移)
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            # 发布变换
            self.tf_broadcaster.sendTransform(t)
            
            if self.counter % self.warning_interval == 0:
                self.get_logger().info('发布静态变换: map -> odom')
        
        # 增加计数器
        self.counter += 1

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 创建节点
    tf_fix_node = TFFixNode()
    
    try:
        # 运行节点
        rclpy.spin(tf_fix_node)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理并关闭节点
        tf_fix_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
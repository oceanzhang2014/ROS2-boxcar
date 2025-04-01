#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简单的关节状态发布器，为小车的轮子发布固定值关节状态
替代joint_state_publisher包，使轮子位置在静止时保持完全稳定
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import threading
import time

class SimpleJointStatePublisher(Node):
    """简单关节状态发布器 - 固定关节位置"""
    
    def __init__(self):
        super().__init__('simple_joint_state_publisher')
        
        # 声明参数
        self.declare_parameter_if_not_declared('publish_rate', 1.0)  # 将发布频率降至最低 - 每秒仅1次！
        self.declare_parameter_if_not_declared('use_fixed_positions', True)  # 使用固定位置而不是变化位置
        
        # 获取参数
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_fixed_positions = self.get_parameter('use_fixed_positions').value
        
        # 创建关节状态发布器
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # 定义关节名称列表 - 所有轮子的关节
        self.joint_names = [
            'base_to_front_left_wheel',
            'base_to_front_right_wheel',
            'base_to_rear_left_wheel',
            'base_to_rear_right_wheel'
        ]
        
        # 使用固定的关节位置值，不再变化
        self.joint_positions = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        
        # 确保即使在ROS系统重启或崩溃的情况下也能发布消息
        self._lock = threading.Lock()
        self._running = True
        
        # 创建定时器和线程
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_state)
        
        # 立即发布一次确保TF树初始化
        self.publish_joint_state()
        
        log_msg = '简单关节状态发布器已启动'
        if self.use_fixed_positions:
            log_msg += '，发布固定轮子位置，确保最高稳定性'
        else:
            log_msg += '，轮子位置可能随cmd_vel变化'
        log_msg += f'，发布频率: {self.publish_rate} Hz'
        
        self.get_logger().info(log_msg)
    
    def declare_parameter_if_not_declared(self, name, value):
        """安全地声明参数，避免重复声明错误"""
        try:
            return self.declare_parameter(name, value)
        except:
            return self.get_parameter(name)
    
    def publish_joint_state(self):
        """发布关节状态"""
        try:
            with self._lock:
                if not self._running:
                    return
                
                # 创建关节状态消息
                joint_state = JointState()
                
                # 设置消息头 - 使用ROS时间
                current_time = self.get_clock().now().to_msg()
                joint_state.header = Header()
                joint_state.header.stamp = current_time
                
                # 设置关节名称
                joint_state.name = self.joint_names
                
                # 设置固定的关节位置（这里使用0.0表示轮子不转动的状态）
                joint_state.position = self.joint_positions
                
                # 设置固定的关节速度（静止）
                joint_state.velocity = self.joint_velocities
                
                # 发布消息
                self.publisher_.publish(joint_state)
        except Exception as e:
            self.get_logger().error(f'发布关节状态时出错: {str(e)}')
    
    def destroy_node(self):
        """安全关闭节点"""
        with self._lock:
            self._running = False
        super().destroy_node()

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    # 创建并运行节点
    node = SimpleJointStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    except Exception as e:
        node.get_logger().error(f'发生异常: {str(e)}')
    finally:
        # 清理资源
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
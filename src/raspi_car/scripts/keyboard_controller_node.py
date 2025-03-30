#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
w

此节点捕获键盘输入并发布Twist消息到cmd_vel话题
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios
import threading
import time

# 定义键盘映射
KEY_MAPPING = {
    'w': [0.2, 0.0],    # 前进
    's': [-0.2, 0.0],   # 后退
    'a': [0.2, 0.5],    # 左转
    'd': [0.2, -0.5],   # 右转
    'q': [0.2, 0.5],    # 左前
    'e': [0.2, -0.5],   # 右前
    'z': [-0.2, 0.5],   # 左后
    'c': [-0.2, -0.5],  # 右后
    ' ': [0.0, 0.0]     # 停止
}

class KeyboardControllerNode(Node):
    """键盘控制ROS2节点"""

    def __init__(self):
        super().__init__('keyboard_controller')
        
        # 创建Twist消息发布者
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        
        # 创建Twist消息
        self.twist_msg = Twist()
        
        # 设置运行标志
        self.running = True
        
        # 记录上次发布的速度
        self.last_linear = 0.0
        self.last_angular = 0.0
        
        # 创建键盘读取线程
        self.keyboard_thread = threading.Thread(target=self.read_keyboard_input)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # 创建消息发布定时器
        self.timer = self.create_timer(0.1, self.publish_cmd_vel)
        
        # 打印控制说明
        self.print_usage()
    
    def print_usage(self):
        """打印键盘控制说明"""
        msg = """
控制小车移动:
----------------------------
   q    w    e
   a    s    d
   z         c

w/s : 前进/后退
a/d : 左转/右转
q/e : 左前/右前
z/c : 左后/右后
空格 : 停止
CTRL+C : 退出

按下按键开始移动
"""
        self.get_logger().info(msg)
    
    def read_keyboard_input(self):
        """读取键盘输入"""
        try:
            # 保存终端设置
            old_settings = termios.tcgetattr(sys.stdin)
            
            # 设置终端为raw模式
            tty.setraw(sys.stdin.fileno())
            
            self.get_logger().info('键盘控制已准备就绪，请按键控制移动...')
            
            while self.running:
                try:
                    # 检查是否有键盘输入
                    if select.select([sys.stdin], [], [], 0)[0]:
                        key = sys.stdin.read(1)
                        
                        # 检查是否为退出命令(CTRL+C)
                        if key == '\x03':
                            self.running = False
                            self.get_logger().info('接收到CTRL+C，正在退出...')
                            break
                        
                        # 映射按键到速度命令
                        if key in KEY_MAPPING:
                            [linear, angular] = KEY_MAPPING[key]
                            self.last_linear = linear
                            self.last_angular = angular
                            self.get_logger().info(f'按键: {key}, 线速度: {linear:.1f}, 角速度: {angular:.1f}')
                    
                    # 短暂休眠，避免消耗过多CPU
                    time.sleep(0.01)
                except Exception as e:
                    self.get_logger().error(f'键盘读取错误: {str(e)}')
                    time.sleep(0.5)  # 错误后短暂暂停
        except Exception as e:
            self.get_logger().error(f'键盘控制初始化错误: {str(e)}')
        finally:
            # 恢复终端设置
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            except:
                self.get_logger().warn('无法恢复终端设置')
    
    def publish_cmd_vel(self):
        """发布速度命令"""
        if not self.running:
            return
        
        # 更新Twist消息
        self.twist_msg.linear.x = self.last_linear
        self.twist_msg.angular.z = self.last_angular
        
        # 发布消息
        self.cmd_vel_publisher.publish(self.twist_msg)
    
    def shutdown(self):
        """关闭节点前的清理工作"""
        # 停止键盘读取线程
        self.running = False
        
        # 发送停止命令
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)
        self.get_logger().info('已发送停止命令')

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    keyboard_controller = None
    
    try:
        keyboard_controller = KeyboardControllerNode()
        
        # 执行节点
        try:
            rclpy.spin(keyboard_controller)
        except KeyboardInterrupt:
            pass
        finally:
            if keyboard_controller:
                keyboard_controller.shutdown()
    except Exception as e:
        print(f"发生错误: {str(e)}")
    finally:
        # 清理资源
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
电机控制节点: 通过PWM控制左右两个电机

此节点订阅cmd_vel话题，控制车轮转速和方向
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import DigitalOutputDevice, PWMOutputDevice
import threading
import time
import signal
import sys

# 定义GPIO引脚
# 左电机
LEFT_MOTOR_ENABLE = 18  # ENA - PWM控制引脚
LEFT_MOTOR_IN1 = 23     # IN1 - 方向控制
LEFT_MOTOR_IN2 = 24     # IN2 - 方向控制

# 右电机
RIGHT_MOTOR_ENABLE = 12  # ENB - PWM控制引脚
RIGHT_MOTOR_IN3 = 25     # IN3 - 方向控制
RIGHT_MOTOR_IN4 = 26     # IN4 - 方向控制

# PWM频率 (Hz) - gpiozero自动设置
# 安全参数
WATCHDOG_TIMEOUT = 0.5  # 看门狗超时时间 (秒)

class MotorController(Node):
    """电机控制ROS2节点"""

    def __init__(self):
        super().__init__('motor_controller')
        
        # 创建Twist订阅者
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # 初始化GPIO设备
        try:
            self.setup_gpio()
            self.get_logger().info('GPIO设备已初始化')
        except Exception as e:
            self.get_logger().error(f'GPIO初始化失败: {str(e)}')
            raise
        
        # 保存当前速度值
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        
        # 最后一次命令时间
        self.last_cmd_time = time.time()
        
        # 看门狗定时器
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
        
        # 注册信号处理函数
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.get_logger().info('电机控制节点已初始化，使用gpiozero库')
        self.get_logger().info('等待速度命令...')
    
    def setup_gpio(self):
        """设置GPIO设备"""
        # 创建PWM输出设备
        self.left_enable = PWMOutputDevice(LEFT_MOTOR_ENABLE)
        self.right_enable = PWMOutputDevice(RIGHT_MOTOR_ENABLE)
        
        # 创建数字输出设备
        self.left_in1 = DigitalOutputDevice(LEFT_MOTOR_IN1)
        self.left_in2 = DigitalOutputDevice(LEFT_MOTOR_IN2)
        self.right_in3 = DigitalOutputDevice(RIGHT_MOTOR_IN3)
        self.right_in4 = DigitalOutputDevice(RIGHT_MOTOR_IN4)
        
        # 初始化为停止状态
        self.left_in1.off()
        self.left_in2.off()
        self.right_in3.off()
        self.right_in4.off()
        self.left_enable.value = 0
        self.right_enable.value = 0
    
    def cmd_vel_callback(self, msg):
        """处理/cmd_vel话题的回调函数"""
        # 提取线速度和角速度
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z
        
        # 更新当前速度
        self.current_linear_speed = linear_speed
        self.current_angular_speed = angular_speed
        
        # 更新命令时间
        self.last_cmd_time = time.time()
        
        # 计算左右电机速度
        left_speed, right_speed = self.calculate_wheel_speeds(linear_speed, angular_speed)
        
        # 控制电机
        self.set_motor_speeds(left_speed, right_speed)
        
        # 记录速度命令
        self.get_logger().debug(f'收到速度命令 - 线速度: {linear_speed:.2f} m/s, 角速度: {angular_speed:.2f} rad/s')
        self.get_logger().debug(f'计算的轮速 - 左: {left_speed:.2f}, 右: {right_speed:.2f}')
    
    def calculate_wheel_speeds(self, linear, angular):
        """计算左右轮的速度
        
        Args:
            linear (float): 线速度 (-1.0 到 1.0)
            angular (float): 角速度 (-1.0 到 1.0)
            
        Returns:
            tuple: (左轮速度, 右轮速度), 范围为 -1.0 到 1.0
        """
        # 简单的差速转向计算
        # 注意: 如果物理轮距不同，可能需要调整此比例
        left_speed = linear - angular
        right_speed = linear + angular
        
        # 限制速度在 -1.0 到 1.0 范围内
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))
        
        return left_speed, right_speed
    
    def set_motor_speeds(self, left_speed, right_speed):
        """设置左右电机的速度和方向
        
        Args:
            left_speed (float): 左电机速度 (-1.0 到 1.0)
            right_speed (float): 右电机速度 (-1.0 到 1.0)
        """
        # 左电机方向控制
        if left_speed > 0:
            # 正向
            self.left_in1.on()
            self.left_in2.off()
        elif left_speed < 0:
            # 反向
            self.left_in1.off()
            self.left_in2.on()
        else:
            # 停止
            self.left_in1.off()
            self.left_in2.off()
        
        # 右电机方向控制
        if right_speed > 0:
            # 正向
            self.right_in3.on()
            self.right_in4.off()
        elif right_speed < 0:
            # 反向
            self.right_in3.off()
            self.right_in4.on()
        else:
            # 停止
            self.right_in3.off()
            self.right_in4.off()
        
        # 设置PWM值 (gpiozero使用0-1范围)
        self.left_enable.value = abs(left_speed)
        self.right_enable.value = abs(right_speed)
    
    def watchdog_callback(self):
        """看门狗定时器回调函数
        
        检查最后一次命令时间，如果超时则停止电机
        """
        current_time = time.time()
        
        # 检查是否超时
        if current_time - self.last_cmd_time > WATCHDOG_TIMEOUT:
            # 如果电机正在运行，则停止并记录日志
            if self.current_linear_speed != 0.0 or self.current_angular_speed != 0.0:
                self.get_logger().info('看门狗超时: 停止电机')
                self.stop_motors()
                self.current_linear_speed = 0.0
                self.current_angular_speed = 0.0
    
    def stop_motors(self):
        """停止所有电机"""
        # 停止方向控制
        self.left_in1.off()
        self.left_in2.off()
        self.right_in3.off()
        self.right_in4.off()
        
        # 停止PWM
        self.left_enable.value = 0
        self.right_enable.value = 0
    
    def signal_handler(self, sig, frame):
        """信号处理函数，用于优雅地关闭节点"""
        self.get_logger().info('收到终止信号，正在关闭节点...')
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        """清理资源"""
        self.get_logger().info('正在清理GPIO资源...')
        
        # 停止电机
        self.stop_motors()
        
        # 关闭所有GPIO设备
        self.left_enable.close()
        self.right_enable.close()
        self.left_in1.close()
        self.left_in2.close()
        self.right_in3.close()
        self.right_in4.close()
        
        self.get_logger().info('资源清理完成')

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        motor_controller = MotorController()
        
        try:
            # 执行节点
            rclpy.spin(motor_controller)
        except KeyboardInterrupt:
            pass
        finally:
            # 清理资源
            motor_controller.cleanup()
            motor_controller.destroy_node()
    except Exception as e:
        print(f"发生错误: {str(e)}")
    finally:
        # 清理ROS资源
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
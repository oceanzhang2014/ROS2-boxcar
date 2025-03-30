#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MPU6050数据订阅器: 订阅并处理MPU6050传感器数据
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
from transforms3d.euler import quat2euler

class MPU6050Subscriber(Node):
    """订阅MPU6050传感器数据的ROS2节点"""

    def __init__(self):
        super().__init__('mpu6050_subscriber')
        
        # 创建订阅者
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.imu_callback,
            10)
        
        self.get_logger().info('MPU6050订阅者已启动，等待数据...')
    
    def imu_callback(self, msg):
        """处理接收到的IMU数据"""
        
        # 提取加速度数据 (m/s^2)
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z
        
        # 提取角速度数据 (rad/s)
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z
        
        # 提取方向四元数
        quat = [
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ]
        
        # 将四元数转换为欧拉角 (弧度)
        roll, pitch, yaw = quat2euler(quat)
        
        # 打印数据
        self.get_logger().info(
            f'\n加速度 (m/s²): X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f}\n'
            f'角速度 (rad/s): X={gyro_x:.2f}, Y={gyro_y:.2f}, Z={gyro_z:.2f}\n'
            f'姿态角 (度): Roll={math.degrees(roll):.2f}, Pitch={math.degrees(pitch):.2f}, Yaw={math.degrees(yaw):.2f}'
        )

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    mpu6050_subscriber = MPU6050Subscriber()
    
    try:
        rclpy.spin(mpu6050_subscriber)
    except KeyboardInterrupt:
        mpu6050_subscriber.get_logger().info('用户中断')
    finally:
        # 清理资源
        mpu6050_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
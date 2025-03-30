#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
IMU与里程计融合节点: 结合MPU6050数据和编码器里程计进行数据融合

此节点订阅IMU原始数据和电机编码器数据，进行数据融合并发布融合后的IMU数据
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from tf2_ros import TransformBroadcaster
from tf2_ros import StaticTransformBroadcaster
import math
import numpy as np
from transforms3d.euler import euler2quat
import time

class IMUOdomFusion(Node):
    """IMU与里程计融合节点"""
    
    def __init__(self):
        super().__init__('imu_odom_fusion')
        
        # 声明参数
        self.declare_parameter_if_not_declared('use_sim_time', False)
        self.declare_parameter_if_not_declared('imu_frame', 'imu_link')
        self.declare_parameter_if_not_declared('base_frame', 'base_link')
        self.declare_parameter_if_not_declared('odom_frame', 'odom')
        self.declare_parameter_if_not_declared('imu_topic', '/imu/data_raw')
        self.declare_parameter_if_not_declared('odom_topic', '/odom')
        self.declare_parameter_if_not_declared('publish_rate', 50.0)  # Hz
        self.declare_parameter_if_not_declared('alpha', 0.95)  # 互补滤波器系数
        
        # 获取参数
        self.imu_frame = self.get_parameter('imu_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.alpha = self.get_parameter('alpha').value
        
        # 创建TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 创建静态TF: base_link -> imu_link
        self.publish_static_transforms()
        
        # 创建订阅者
        self.imu_subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10)
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10)
        
        # 创建发布者
        self.fused_imu_publisher = self.create_publisher(
            Imu,
            '/imu/data',
            10)
        
        # 状态变量
        self.last_imu = None
        self.last_odom = None
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        
        # 融合结果
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # 记录上次警告的时间，用于限制警告频率
        self.last_warning_time = 0.0
        
        # 创建定时器发布融合数据
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_fused_data)
        
        self.get_logger().info('IMU与里程计融合节点已启动')
        self.get_logger().info(f'订阅IMU主题: {self.imu_topic}')
        self.get_logger().info(f'订阅里程计主题: {self.odom_topic}')
        self.get_logger().info(f'发布频率: {self.publish_rate} Hz')
    
    def declare_parameter_if_not_declared(self, name, value):
        """安全地声明参数，避免重复声明错误"""
        try:
            return self.declare_parameter(name, value)
        except Exception as e:
            self.get_logger().debug(f'参数 {name} 已经被声明或声明失败: {str(e)}')
            return self.get_parameter(name)
    
    def publish_static_transforms(self):
        """发布静态TF变换"""
        try:
            # 发布 base_link -> imu_link 变换
            imu_transform = TransformStamped()
            imu_transform.header.stamp = self.get_clock().now().to_msg()
            imu_transform.header.frame_id = self.base_frame
            imu_transform.child_frame_id = self.imu_frame
            
            # 设置IMU相对于底盘的位置 (在车辆中心偏上方)
            imu_transform.transform.translation.x = 0.0
            imu_transform.transform.translation.y = 0.0
            imu_transform.transform.translation.z = 0.05
            
            # 设置IMU相对于底盘的朝向 (与底盘同向)
            q = euler2quat(0.0, 0.0, 0.0, 'sxyz')
            imu_transform.transform.rotation.x = q[1]
            imu_transform.transform.rotation.y = q[2]
            imu_transform.transform.rotation.z = q[3]
            imu_transform.transform.rotation.w = q[0]
            
            # 发布静态变换
            self.static_tf_broadcaster.sendTransform(imu_transform)
            
            self.get_logger().info(f'已发布静态变换: {self.base_frame} -> {self.imu_frame}')
        except Exception as e:
            self.get_logger().error(f'发布静态变换时出错: {str(e)}')
    
    def imu_callback(self, msg):
        """IMU数据回调处理"""
        self.last_imu = msg
    
    def odom_callback(self, msg):
        """里程计数据回调处理"""
        self.last_odom = msg
    
    def warning_throttle(self, interval, msg):
        """限制警告日志的频率"""
        current_time = time.time()
        if current_time - self.last_warning_time >= interval:
            self.get_logger().warning(msg)
            self.last_warning_time = current_time
    
    def publish_fused_data(self):
        """发布融合后的IMU数据"""
        # 如果没有收到IMU或里程计数据，则返回
        if self.last_imu is None:
            self.warning_throttle(5, '尚未接收到IMU数据')
            return
        
        # 创建融合后的IMU消息
        fused_imu = Imu()
        fused_imu.header.stamp = self.get_clock().now().to_msg()
        fused_imu.header.frame_id = self.imu_frame
        
        # 当前时间
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 提取IMU数据
        accel = self.last_imu.linear_acceleration
        gyro = self.last_imu.angular_velocity
        
        # 从加速度计计算倾角
        accel_roll = math.atan2(accel.y, math.sqrt(accel.x**2 + accel.z**2))
        accel_pitch = math.atan2(-accel.x, math.sqrt(accel.y**2 + accel.z**2))
        
        # 使用陀螺仪积分得到角度变化
        gyro_roll = self.roll + gyro.x * dt
        gyro_pitch = self.pitch + gyro.y * dt
        
        # 互补滤波
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        
        # 从里程计获取偏航角
        if self.last_odom is not None:
            # 从里程计四元数提取偏航角
            q = self.last_odom.pose.pose.orientation
            _, _, self.yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        else:
            # 如果没有里程计数据，使用陀螺仪积分更新偏航角
            self.yaw = self.yaw + gyro.z * dt
        
        # 将欧拉角转换为四元数
        q = euler2quat(self.roll, self.pitch, self.yaw, 'sxyz')
        
        # 设置方向四元数
        fused_imu.orientation.w = q[0]
        fused_imu.orientation.x = q[1]
        fused_imu.orientation.y = q[2]
        fused_imu.orientation.z = q[3]
        
        # 设置角速度和加速度
        fused_imu.angular_velocity = gyro
        fused_imu.linear_acceleration = accel
        
        # 设置协方差
        fused_imu.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        fused_imu.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        fused_imu.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        
        # 发布融合后的IMU数据
        self.fused_imu_publisher.publish(fused_imu)
    
    def euler_from_quaternion(self, x, y, z, w):
        """
        将四元数转换为欧拉角
        
        Args:
            x, y, z, w: 四元数分量
        
        Returns:
            roll, pitch, yaw: 欧拉角
        """
        # 计算欧拉角
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return roll, pitch, yaw

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        fusion_node = IMUOdomFusion()
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'fusion_node' in locals():
            fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
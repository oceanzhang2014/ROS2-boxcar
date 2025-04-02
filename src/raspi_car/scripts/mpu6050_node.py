#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MPU6050 ROS2节点: 读取MPU6050传感器数据并发布为ROS消息

此节点读取MPU6050加速度计和陀螺仪数据，并发布为标准的ROS2 IMU消息
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time

import time
import threading
import math
import numpy as np
from transforms3d.euler import euler2quat

# 移除对test_mpu6050的导入
import sys
import os
import smbus2 as smbus

# MPU6050寄存器地址
PWR_MGMT_1 = 0x6B      # 电源管理寄存器1
WHO_AM_I = 0x75        # WHO_AM_I寄存器，用于验证设备ID
SMPLRT_DIV = 0x19      # 采样率分频器
CONFIG = 0x1A          # 配置寄存器
GYRO_CONFIG = 0x1B     # 陀螺仪配置寄存器
ACCEL_CONFIG = 0x1C    # 加速度计配置寄存器
INT_ENABLE = 0x38      # 中断使能寄存器

# 数据寄存器
ACCEL_XOUT_H = 0x3B    # 加速度计X轴高字节
ACCEL_YOUT_H = 0x3D    # 加速度计Y轴高字节
ACCEL_ZOUT_H = 0x3F    # 加速度计Z轴高字节
TEMP_OUT_H = 0x41      # 温度高字节
TEMP_OUT_L = 0x42      # 温度低字节
GYRO_XOUT_H = 0x43     # 陀螺仪X轴高字节
GYRO_YOUT_H = 0x45     # 陀螺仪Y轴高字节
GYRO_ZOUT_H = 0x47     # 陀螺仪Z轴高字节

# 陀螺仪和加速度计量程
GYRO_SCALE = 131.0     # 陀螺仪满量程为±250°/s时的灵敏度，单位为LSB/(°/s)
ACCEL_SCALE = 16384.0  # 加速度计满量程为±2g时的灵敏度，单位为LSB/g

class MPU6050:
    """MPU6050传感器接口类"""
    
    def __init__(self, address=0x68, bus_num=1):
        """
        初始化MPU6050对象
        
        Args:
            address: I2C设备地址，通常为0x68或0x69
            bus_num: I2C总线号
        """
        self.address = address
        self.bus_num = bus_num
        self.bus = smbus.SMBus(bus_num)
        
        # 初始化传感器
        self.initialize()
        
        # 校准偏移量
        self.accel_offset = {"x": 0, "y": 0, "z": 0}
        self.gyro_offset = {"x": 0, "y": 0, "z": 0}
    
    def initialize(self):
        """初始化MPU6050传感器配置"""
        # 唤醒MPU6050
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        
        # 设置采样率分频
        self.bus.write_byte_data(self.address, SMPLRT_DIV, 0x07)
        
        # 设置低通滤波器
        self.bus.write_byte_data(self.address, CONFIG, 0x06)
        
        # 设置陀螺仪量程为±250°/s
        self.bus.write_byte_data(self.address, GYRO_CONFIG, 0x00)
        
        # 设置加速度计量程为±2g
        self.bus.write_byte_data(self.address, ACCEL_CONFIG, 0x00)
    
    def read_raw_data(self, reg):
        """
        读取MPU6050寄存器的原始数据
        
        Args:
            reg: 寄存器地址
            
        Returns:
            读取的16位有符号整数
        """
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        
        # 合并高位和低位字节
        value = (high << 8) | low
        
        # 处理有符号整数（补码形式）
        if value > 32767:
            value = value - 65536
            
        return value
    
    def read_accel(self):
        """
        读取加速度计数据
        
        Returns:
            包含x、y、z轴加速度数据的字典，单位为g
        """
        x = self.read_raw_data(ACCEL_XOUT_H) / ACCEL_SCALE - self.accel_offset["x"]
        y = self.read_raw_data(ACCEL_YOUT_H) / ACCEL_SCALE - self.accel_offset["y"]
        z = self.read_raw_data(ACCEL_ZOUT_H) / ACCEL_SCALE - self.accel_offset["z"]
        
        return {"x": x, "y": y, "z": z}
    
    def read_gyro(self):
        """
        读取陀螺仪数据
        
        Returns:
            包含x、y、z轴角速度数据的字典，单位为°/s
        """
        x = self.read_raw_data(GYRO_XOUT_H) / GYRO_SCALE - self.gyro_offset["x"]
        y = self.read_raw_data(GYRO_YOUT_H) / GYRO_SCALE - self.gyro_offset["y"]
        z = self.read_raw_data(GYRO_ZOUT_H) / GYRO_SCALE - self.gyro_offset["z"]
        
        return {"x": x, "y": y, "z": z}
    
    def read_temp(self):
        """
        读取温度数据
        
        Returns:
            温度，单位为摄氏度
        """
        raw_temp = self.read_raw_data(TEMP_OUT_H)
        temp_celsius = raw_temp / 340.0 + 36.53
        
        return temp_celsius
    
    def calibrate(self, samples=100):
        """
        校准MPU6050
        
        通过读取多个样本来确定偏移量
        
        Args:
            samples: 用于校准的样本数
        """
        print("开始校准MPU6050，保持传感器静止...")
        
        # 初始化累加器
        accel_sum = {"x": 0, "y": 0, "z": 0}
        gyro_sum = {"x": 0, "y": 0, "z": 0}
        
        # 收集样本
        for _ in range(samples):
            # 读取原始数据（不应用当前偏移量）
            accel_raw = {
                "x": self.read_raw_data(ACCEL_XOUT_H) / ACCEL_SCALE,
                "y": self.read_raw_data(ACCEL_YOUT_H) / ACCEL_SCALE,
                "z": self.read_raw_data(ACCEL_ZOUT_H) / ACCEL_SCALE
            }
            
            gyro_raw = {
                "x": self.read_raw_data(GYRO_XOUT_H) / GYRO_SCALE,
                "y": self.read_raw_data(GYRO_YOUT_H) / GYRO_SCALE,
                "z": self.read_raw_data(GYRO_ZOUT_H) / GYRO_SCALE
            }
            
            # 累加样本
            for axis in ["x", "y", "z"]:
                accel_sum[axis] += accel_raw[axis]
                gyro_sum[axis] += gyro_raw[axis]
            
            time.sleep(0.005)  # 短暂延迟
        
        # 计算平均值作为偏移量
        for axis in ["x", "y", "z"]:
            # 陀螺仪偏移量直接是平均值
            self.gyro_offset[axis] = gyro_sum[axis] / samples
            
            # 加速度计Z轴应该是1g，其他轴为0g
            if axis == "z":
                self.accel_offset[axis] = (accel_sum[axis] / samples) - 1.0
            else:
                self.accel_offset[axis] = accel_sum[axis] / samples
        
        print("校准完成")
        print(f"加速度计偏移量: {self.accel_offset}")
        print(f"陀螺仪偏移量: {self.gyro_offset}")
    
    def get_rotation(self):
        """
        计算当前姿态角（欧拉角）
        
        Returns:
            包含roll、pitch角度的字典，单位为度
        """
        accel = self.read_accel()
        
        # 基于加速度计计算姿态角
        roll = math.atan2(accel["y"], math.sqrt(accel["x"]**2 + accel["z"]**2)) * 180 / math.pi
        pitch = math.atan2(-accel["x"], math.sqrt(accel["y"]**2 + accel["z"]**2)) * 180 / math.pi
        
        return {"roll": roll, "pitch": pitch}

class MPU6050Node(Node):
    """MPU6050 ROS2节点"""
    
    def __init__(self):
        super().__init__('mpu6050_node')
        
        # 创建MPU6050对象
        self.mpu = None
        
        # 创建IMU消息发布者
        self.imu_publisher = self.create_publisher(
            Imu,
            'imu/data_raw',
            10)
        
        # 声明参数
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('bus_num', 1)
        self.declare_parameter('device_addr', 0x68)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('ignore_connection_errors', False)  # 添加忽略连接错误参数
        
        # 获取参数
        self.frame_id = self.get_parameter('frame_id').value
        self.bus_num = self.get_parameter('bus_num').value
        self.device_addr = self.get_parameter('device_addr').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.ignore_connection_errors = self.get_parameter('ignore_connection_errors').value
        
        # 创建互斥锁
        self.lock = threading.Lock()
        
        # 补偿滤波参数
        self.alpha = 0.98  # 互补滤波系数
        self.last_time = time.time()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # 初始化MPU6050
        try:
            self.mpu = MPU6050(address=self.device_addr, bus_num=self.bus_num)
            self.get_logger().info('MPU6050初始化成功')
            
            # 校准传感器
            self.calibrate_sensor()
        except Exception as e:
            self.get_logger().error(f'MPU6050初始化失败: {str(e)}')
            return
        
        # 创建定时器
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_imu_data)
        
        self.get_logger().info(
            f'MPU6050节点已启动，发布频率: {self.publish_rate} Hz, 话题: imu/data_raw')
    
    def calibrate_sensor(self):
        """校准传感器"""
        self.get_logger().info('开始校准MPU6050，请保持传感器静止...')
        
        # 加上锁防止在校准过程中读取数据
        with self.lock:
            # 执行校准
            self.mpu.calibrate(samples=100)
        
        self.get_logger().info('MPU6050校准完成')
    
    def publish_imu_data(self):
        """读取MPU6050数据并发布"""
        # 如果MPU6050未初始化，则返回
        if self.mpu is None:
            self.get_logger().error_once('MPU6050未初始化，无法发布数据')
            return
        
        try:
            # 使用锁保护传感器读取操作
            with self.lock:
                # 读取传感器数据
                accel = self.mpu.read_accel()
                gyro = self.mpu.read_gyro()
            
            # 创建IMU消息
            imu_msg = Imu()
            
            # 设置消息头
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # 设置线性加速度 (单位: m/s^2)
            imu_msg.linear_acceleration.x = accel["x"] * 9.81  # 转换为m/s^2
            imu_msg.linear_acceleration.y = accel["y"] * 9.81
            imu_msg.linear_acceleration.z = accel["z"] * 9.81
            
            # 设置角速度 (单位: rad/s)
            imu_msg.angular_velocity.x = math.radians(gyro["x"])  # 转换为rad/s
            imu_msg.angular_velocity.y = math.radians(gyro["y"])
            imu_msg.angular_velocity.z = math.radians(gyro["z"])
            
            # 计算方向四元数
            roll_rad, pitch_rad, yaw_rad = self.update_orientation(accel, gyro)
            
            # 将欧拉角转换为四元数 (w, x, y, z)
            quat = euler2quat(roll_rad, pitch_rad, yaw_rad, 'sxyz')
            
            # 设置方向四元数
            imu_msg.orientation.w = quat[0]
            imu_msg.orientation.x = quat[1]
            imu_msg.orientation.y = quat[2]
            imu_msg.orientation.z = quat[3]
            
            # 设置协方差（这里简化处理，实际应用中应根据传感器特性设置）
            for i in range(9):
                imu_msg.orientation_covariance[i] = 0.01 if i % 4 == 0 else 0.0
                imu_msg.angular_velocity_covariance[i] = 0.01 if i % 4 == 0 else 0.0
                imu_msg.linear_acceleration_covariance[i] = 0.01 if i % 4 == 0 else 0.0
            
            # 发布消息
            self.imu_publisher.publish(imu_msg)
            
        except Exception as e:
            self.get_logger().error(f'读取或发布MPU6050数据时出错: {str(e)}')
    
    def update_orientation(self, accel, gyro):
        """使用互补滤波更新方向估计
        
        结合加速度计和陀螺仪数据，计算更准确的姿态角
        
        Args:
            accel: 加速度数据
            gyro: 角速度数据
            
        Returns:
            更新后的欧拉角（roll, pitch, yaw），单位为弧度
        """
        # 计算采样时间间隔
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 从加速度计计算姿态角（静态）
        accel_roll = math.atan2(accel["y"], math.sqrt(accel["x"]**2 + accel["z"]**2))
        accel_pitch = math.atan2(-accel["x"], math.sqrt(accel["y"]**2 + accel["z"]**2))
        
        # 使用陀螺仪数据更新角度（动态）
        gyro_roll = self.roll + math.radians(gyro["x"]) * dt
        gyro_pitch = self.pitch + math.radians(gyro["y"]) * dt
        gyro_yaw = self.yaw + math.radians(gyro["z"]) * dt
        
        # 互补滤波，结合静态和动态测量
        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        self.yaw = gyro_yaw  # 偏航角只能通过陀螺仪积分或磁力计获得
        
        # 限制偏航角在-pi到pi范围内
        self.yaw = ((self.yaw + math.pi) % (2 * math.pi)) - math.pi
        
        return self.roll, self.pitch, self.yaw

    def publish_fake_imu_data(self):
        """发布模拟IMU数据（全零值）"""
        msg = Imu()
        
        # 设置消息头
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # 设置线性加速度（默认为0）
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81  # 只设置Z轴重力加速度
        
        # 设置角速度（默认为0）
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        
        # 设置协方差 - 设置为高不确定性
        msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]
        
        msg.angular_velocity_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]
        
        # 发布消息
        self.imu_publisher.publish(msg)
        self.get_logger().debug("已发布模拟IMU数据")

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = MPU6050Node()
    
    try:
        # 初始化传感器
        try:
            node.get_logger().info(f"正在尝试连接MPU6050，总线: {node.bus_num}, 设备地址: 0x{node.device_addr:02x}")
            node.mpu = MPU6050(address=node.device_addr, bus_num=node.bus_num)
            node.get_logger().info("MPU6050初始化成功")
            
            # 校准传感器
            node.calibrate_sensor()
            
            # 创建定时器，定期发布IMU数据
            node.timer = node.create_timer(1.0/node.publish_rate, node.publish_imu_data)
            
            node.get_logger().info(f"MPU6050节点已启动，发布频率: {node.publish_rate} Hz")
            
        except Exception as e:
            if node.ignore_connection_errors:
                node.get_logger().warning(f"MPU6050初始化失败: {e}")
                node.get_logger().warning("已启用忽略连接错误模式，将发布模拟IMU数据")
                
                # 发布零值IMU数据
                node.timer = node.create_timer(1.0/node.publish_rate, node.publish_fake_imu_data)
            else:
                node.get_logger().error(f"MPU6050初始化失败: {e}")
                return
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
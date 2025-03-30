#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
URDF发布节点：负责加载和发布机器人模型

此节点读取URDF文件并将其加载到参数服务器中，
使RViz能够正确显示机器人模型
"""

import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET

class URDFPublisherNode(Node):
    """URDF发布节点"""

    def __init__(self):
        super().__init__('urdf_publisher')
        
        # 声明参数
        self.declare_parameter('urdf_file', 'raspi_car.urdf')
        
        # 获取参数
        urdf_file = self.get_parameter('urdf_file').value
        
        # 获取包路径
        package_path = get_package_share_directory('raspi_car')
        urdf_path = os.path.join(package_path, 'urdf', urdf_file)
        
        # 读取URDF文件
        try:
            with open(urdf_path, 'r') as file:
                robot_description = file.read()
                
            # 发布URDF到参数服务器
            self.declare_parameter('robot_description', robot_description)
            
            self.get_logger().info(f'已加载URDF模型: {urdf_path}')
            
            # 解析URDF获取关节名称
            root = ET.fromstring(robot_description)
            joints = root.findall(".//joint")
            joint_names = [joint.attrib['name'] for joint in joints]
            
            self.get_logger().info(f'模型包含以下关节: {", ".join(joint_names)}')
            
        except Exception as e:
            self.get_logger().error(f'加载URDF失败: {str(e)}')

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    urdf_publisher = URDFPublisherNode()
    
    try:
        rclpy.spin(urdf_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        urdf_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
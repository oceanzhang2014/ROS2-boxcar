from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'raspi_car'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages() + ['scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加启动文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 添加配置文件
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # 添加URDF文件
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # 添加脚本文件到可执行目录
        ('lib/' + package_name, glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ocean',
    maintainer_email='ocean@example.com',
    description='ROS 2 package for Raspberry Pi car with SiLan C1 Lidar',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_key = raspi_car.teleop_key:main',
            'car_controller = raspi_car.car_controller:main',
            'silan_c1_node = raspi_car.silan_c1_node:main',
            'test_silan_c1 = raspi_car.test_silan_c1:main',
            # 停止和重置工具
            'stop_silan_c1 = raspi_car.stop_silan_c1:main',
            # 新增思岚C1支持
            'slamtec_c1_node = raspi_car.slamtec_c1_node:main',
            'test_slamtec_c1 = raspi_car.test_slamtec_c1:main',
            'stop_slamtec_c1 = raspi_car.stop_slamtec_c1:main',
            # 可视化工具
            'visualize_lidar = raspi_car.visualize_lidar:main',
            # scripts目录中的节点
            'mpu6050_node = scripts.mpu6050_node:main',
            'motor_controller_node = scripts.motor_controller_node:main',
            'keyboard_controller_node = scripts.keyboard_controller_node:main',
            'tf_publisher_node = scripts.tf_publisher_node:main',
            'urdf_publisher_node = scripts.urdf_publisher_node:main',
            'fix_tf_chain = scripts.fix_tf_chain:main',
            'mpu6050_subscriber = scripts.mpu6050_subscriber:main',
            # 里程计和传感器融合节点
            'encoder_odom_node = scripts.encoder_odom_node:main',
            'imu_odom_fusion = scripts.imu_odom_fusion:main',
            # 关节状态发布节点
            'joint_state_publisher_node = scripts.joint_state_publisher_node:main',
        ],
    },
) 
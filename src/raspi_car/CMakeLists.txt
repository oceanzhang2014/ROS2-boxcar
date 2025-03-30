cmake_minimum_required(VERSION 3.8)
project(raspi_car)

# 使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装config文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 安装maps目录
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}
)

# 安装urdf目录
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# 安装Python模块
ament_python_install_package(${PROJECT_NAME})

# 安装Python脚本
install(PROGRAMS
  scripts/mpu6050_node.py
  scripts/mpu6050_subscriber.py
  scripts/motor_controller_node.py
  scripts/keyboard_controller_node.py
  scripts/tf_publisher_node.py
  scripts/fix_tf_chain.py
  scripts/imu_fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(rclpy sensor_msgs geometry_msgs tf2 tf2_ros std_msgs nav_msgs)
ament_package() 
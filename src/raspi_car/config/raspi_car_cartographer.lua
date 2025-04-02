-- Copyright 2023 RaspberryPi Car Team
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "laser_stable",  -- 使用稳定激光雷达坐标系作为跟踪坐标系，确保数据点方向稳定
  published_frame = "base_link",  -- 使用base_link，避免与odom_frame冲突
  odom_frame = "odom",
  provide_odom_frame = true,  -- 让Cartographer提供map->odom变换
  publish_frame_projected_to_2d = true,  -- 保证在2D平面上的投影
  use_pose_extrapolator = true,  -- 使用位姿外推器
  use_odometry = false,  -- 禁用里程计，因为没有可靠的IMU数据
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,  -- 超时时间
  submap_publish_period_sec = 3.0,  -- 增加子地图发布周期，减少更新频率
  pose_publish_period_sec = 50e-3,  -- 增加位姿发布周期，减少TF树更新频率
  trajectory_publish_period_sec = 100e-3,  -- 增加轨迹发布周期，减少更新频率
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,  -- 使用全部IMU数据
  landmarks_sampling_ratio = 1.0,
  publish_to_tf = true,  -- 必须发布到TF
  publish_tracked_pose = true,  -- 发布跟踪的位姿
}

-- 调整2D轨迹构建器参数
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 在使用laser_stable作为tracking_frame时，我们禁用IMU数据
TRAJECTORY_BUILDER_2D.min_range = 0.2  -- 最小测量距离
TRAJECTORY_BUILDER_2D.max_range = 8.0  -- 最大测量距离，确保能捕捉更远的特征
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 启用在线相关扫描匹配

-- 设置Ceres扫描匹配器参数 - 调整为更合理的值
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2.0  -- 恢复到更合理的平移权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0  -- 降低到更合理的旋转权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 40.0  -- 设置为更合理的占用空间权重

-- 运动过滤器参数 - 调整为更少频繁的更新
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 10.0  -- 增加到10秒，极大减少静止状态下的更新频率
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.3  -- 增加距离阈值，减少小幅移动触发的更新
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(8.0)  -- 增加角度阈值，减少小角度旋转触发的更新

-- Pose图优化参数
POSE_GRAPH.optimize_every_n_nodes = 100  -- 设置更合理的优化频率
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 调整到更合理的阈值
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65  -- 与上方保持一致

-- 增强角度优化
POSE_GRAPH.matcher_translation_weight = 1.0  -- 更合理的平移权重
POSE_GRAPH.matcher_rotation_weight = 10.0  -- 更合理的旋转权重

-- 位置图的优化选项 - 使用更合理的值
POSE_GRAPH.optimization_problem.huber_scale = 1e2  -- 降低到更合理的值
POSE_GRAPH.optimization_problem.acceleration_weight = 1e1  -- 设置为更合理的加速度权重
POSE_GRAPH.optimization_problem.rotation_weight = 1e5  -- 降低到更合理的旋转权重
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5  -- 调整到更合理的值
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0  -- 设置为更合理的约束距离

-- 回环检测参数 - 使用更合理的值
POSE_GRAPH.constraint_builder.sampling_ratio = 0.2  -- 设置为更合理的采样率
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.0
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5  -- 设置为更合理的值
POSE_GRAPH.constraint_builder.log_matches = true  -- 记录匹配情况，方便调试

-- 子图构建参数
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120  -- 设置为更合理的值
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05  -- 保持地图分辨率

-- 点云过滤参数 - 使用更合理的值
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.03  -- 设置为更合理的体素滤波大小
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5  -- 设置为更合理的值
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200  -- 设置为更合理的值
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 6.0  -- 设置为更合理的值

-- 本地SLAM后端
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 15  -- 设置为更合理的迭代次数
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1  -- 单线程，适合RPi

-- IMU姿态相关参数
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0  -- 设置为更合理的值

return options
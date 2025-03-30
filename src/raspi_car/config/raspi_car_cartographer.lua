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
  tracking_frame = "imu_link", -- 使用IMU作为跟踪帧以提高精度
  published_frame = "odom",     -- 发布帧：使用里程计
  odom_frame = "odom",
  provide_odom_frame = false,   -- 我们已经有自己的里程计
  publish_frame_projected_to_2d = true,
  
  -- 增加时间缓冲，解决树莓派处理延迟问题
  use_pose_extrapolator = true,
  use_odometry = true,          -- 使用里程计数据
  use_nav_sat = false,          -- 不使用GPS
  use_landmarks = false,        -- 不使用地标
  num_laser_scans = 1,         -- 使用激光扫描
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  
  -- 增加超时时间，适应树莓派处理能力
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,       -- 降低子地图发布频率，节约资源
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  
  -- 数据采样率 - 保持全部激光数据以提高建图质量
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,     -- 保持使用全部IMU数据
  landmarks_sampling_ratio = 1.0,
}

-- 使用2D轨迹构建器
MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D SLAM优化参数
TRAJECTORY_BUILDER_2D.min_range = 0.15 -- 激光雷达最小范围（米）
TRAJECTORY_BUILDER_2D.max_range = 12.0 -- 激光雷达最大范围（米）
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0 -- 当激光点超出范围时的填充距离
TRAJECTORY_BUILDER_2D.use_imu_data = true -- 启用IMU数据进行姿态和方向校正
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2e2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 4e2

-- 增加IMU权重，在转弯时提高准确性
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5) -- 降低角度阈值以捕获更多转弯

-- 为树莓派优化的参数
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35 -- 增加每个子地图中的扫描数量，提高精度
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05 -- 栅格地图分辨率（米/像素）

-- 正确的range_data_inserter配置
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter = {
  range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
  probability_grid_range_data_inserter = {
    insert_free_space = true,
    hit_probability = 0.55,
    miss_probability = 0.49,
  },
  tsdf_range_data_inserter = {
    truncation_distance = 0.3,
    maximum_weight = 10.0,
    update_free_space = false,
    normal_estimation_options = {
      num_normal_samples = 4,
      sample_radius = 0.5,
    },
    project_sdf_distance_to_scan_normal = true,
    update_weight_range_exponent = 0,
    update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
    update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
  }
}

-- 优化IMU参数以处理小车的快速转弯和抖动
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0  -- 增加重力时间常数以平滑IMU数据

-- 对于树莓派有限资源，降低计算复杂度
POSE_GRAPH.optimize_every_n_nodes = 20 -- 降低优化周期，提高实时性
POSE_GRAPH.constraint_builder.min_score = 0.6 -- 稍微降低最小约束分数，增加回环检测机会
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65 -- 全局定位最小分数

-- 其他性能优化
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3 -- 降低约束采样率，加快处理速度
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0 -- 约束搜索距离
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
POSE_GRAPH.matcher_translation_weight = 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3

-- 降低优化时的计算负担
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)

-- 改进实时性能设置
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5 -- 体素过滤器最大长度
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 150 -- 减少最小点数

-- 明确设置栅格类型 - 这是必需的，防止错误
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"

return options 
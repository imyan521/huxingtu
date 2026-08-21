---- Copyright 2025 Cartographer Android
--include "map_builder.lua"
--include "trajectory_builder.lua"
--
--options = {
--  map_builder = MAP_BUILDER,
--  trajectory_builder = TRAJECTORY_BUILDER,
--
--  -- 🚨 必须保留这些基础 Key，底层 C++ 库加载器需要读取它们以完成初始化 🚨
--  map_frame = "map",
--  tracking_frame = "imu",      -- 根据你之前的参数，这里设为 imu
--  published_frame = "base_link",
--  odom_frame = "odom",
--  provide_odom_frame = true,
--  publish_frame_projected_to_2d = true,
--  use_pose_extrapolator = true,
--  use_odometry = false,
--  use_nav_sat = false,
--  use_landmarks = false,
--  num_laser_scans = 1,
--  num_multi_echo_laser_scans = 0,
--  num_subdivisions_per_laser_scan = 1,
--  num_point_clouds = 0,
--  lookup_transform_timeout_sec = 0.2,
--  submap_publish_period_sec = 0.3,
--  pose_publish_period_sec = 5e-3,
--  trajectory_publish_period_sec = 30e-3,
--  rangefinder_sampling_ratio = 1.,
--  odometry_sampling_ratio = 1.,
--  fixed_frame_pose_sampling_ratio = 1.,
--  imu_sampling_ratio = 1.,
--  landmarks_sampling_ratio = 1.,
--} -- 🌟 关键：表定义在这里结束
--
---- ✅ 在这里修改你关心的核心算法参数
--MAP_BUILDER.use_trajectory_builder_2d = true
--TRAJECTORY_BUILDER_2D.use_imu_data = flase
--TRAJECTORY_BUILDER_2D.min_range = 0.1
--TRAJECTORY_BUILDER_2D.max_range = 15.0
--TRAJECTORY_BUILDER_2D.missing_data_ray_length = 15.0
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20.
--TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0
--return options

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "imu",          -- 必须是 imu
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,

  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ==============================================
-- 🔥 🔥 🔥 这里必须打开 IMU 🔥 🔥 🔥
-- ==============================================
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 15.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 15.0

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 2.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 2.0

return options

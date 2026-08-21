include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",
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
  publish_to_tf = true,
  tf_publish_rate = 20.0,
  scanner_chrome_mode = false,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.05
-- Complex indoor scenes contain long corridor/glass reflections. Restrict
-- matching to the range where the D6 returns are consistently useful.
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.18
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 28
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.25
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 28
-- A finer probability grid and slightly stronger hit/miss contrast produce a
-- narrower, less blocky wall band without post-processing it into a floorplan.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.04
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.58
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.48
-- Longer overlapping submaps reduce room-to-room angular drift without making
-- mobile loop closure excessively expensive.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.10
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)
TRAJECTORY_BUILDER_2D.max_acceleration_range = 10.0
TRAJECTORY_BUILDER_2D.max_angular_velocity_range = 10.0
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher_occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher_translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher_rotation_weight = 20.0

-- Keep already-built rooms visually aligned while mapping instead of waiting
-- for a large batch or final optimization.
POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.55
POSE_GRAPH.global_sampling_ratio = 0.05
POSE_GRAPH.optimization_problem.huber_scale = 1e1

return options

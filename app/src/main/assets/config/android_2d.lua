include "map_builder.lua"
include "trajectory_builder.lua"

-- Android has no TF tree. This is the fixed transform from the lidar frame to
-- the Cartographer tracking frame. Units: meters and radians.
-- With use_imu_data=false, tracking_frame is base_link, so this means
-- lidar -> base_link. With use_imu_data=true, set tracking_frame="imu" and make
-- this lidar -> imu.
lidar_to_tracking = {
  translation = { x = 0.0, y = 0.0, z = 0.0 },
  rotation = { 0.0, 0.0, 0.0 },
}

-- Android sensor axes -> Cartographer tracking frame. Translation is ignored
-- for IMU vectors; keep it zero. Adjust rotation when the phone/IMU axes do
-- not match the robot tracking frame.
imu_to_tracking = {
  translation = { x = 0.0, y = 0.0, z = 0.0 },
  rotation = { 0.0, 0.0, 0.0 },
}

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  -- IMU 模式：
  tracking_frame = "imu",
  -- 纯 LiDAR 测试模式：
  -- tracking_frame = "base_link",
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
-- Constraint building and pose-graph optimization must not starve the USB,
-- IMU and local-SLAM threads on Android. Three workers retain parallel loop
-- closure while leaving a core available for real-time sensor processing.
MAP_BUILDER.num_background_threads = 3

-- IMU 模式：
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- 纯 LiDAR 测试模式：
-- TRAJECTORY_BUILDER_2D.use_imu_data = false
-- Preserve the same useful range as the reference data, but do not turn
-- out-of-range returns into long free-space rays. Those rays enlarge every
-- submap and create the radial gray streaks seen around the saved maps.
TRAJECTORY_BUILDER_2D.min_range = 0.10
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 0.0

-- The reference pbstream retains about 150 filtered points per node, while
-- the previous mobile profile retained only 60-80. Keep the initial voxel
-- filter fine enough for door frames and let the adaptive target control the
-- final scan-matching cost.
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.14
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 30.0
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.90
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50.0

-- Smaller submaps keep local yaw error from being baked into a large section
-- of the building and give final loop-closure optimization more movable units.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
-- Require fewer repeated hits to form a stable wall while leaving free-space
-- clearing conservative. The scan-integrity filters run before insertion, so
-- this strengthens real walls without strengthening partial-scan streaks.
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.58
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49
TRAJECTORY_BUILDER_2D.max_acceleration_range = 10.0
TRAJECTORY_BUILDER_2D.max_angular_velocity_range = 10.0
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- Timestamp/deskew validation is now in the parser, so use the tighter local
-- search profile measured in the clean reference pbstream. This makes a scan
-- less likely to jump to a similar parallel wall. Stronger delta penalties
-- still allow normal handheld motion while rejecting implausible frame jumps.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.10
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(2.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 5.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 5.0

-- Give occupied wall evidence and heading consistency the same authority that
-- produced the cleaner reference map instead of over-constraining translation.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
-- Avoid flooding the pose graph with phone vibration while retaining genuine
-- corner motion. Distance and time branches continue to create straight nodes.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.15
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.7)

-- Correct accumulated error before it spans several rooms. More candidates
-- are evaluated, but stricter scores reject look-alike corridor constraints.
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.58
-- Keep frozen-map relocalization reachable for supplement collection; the
-- normal same-trajectory loop closures still use the stricter min_score above.
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.32
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 8
-- Until a new trajectory connects to a loaded frozen map, every eligible node
-- gets a global match attempt. After the trajectories connect, Cartographer
-- returns to the normal local constraint sampling path above.
POSE_GRAPH.global_sampling_ratio = 1.0
POSE_GRAPH.global_constraint_search_after_n_seconds = 3.0
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.max_num_final_iterations = 300

return options

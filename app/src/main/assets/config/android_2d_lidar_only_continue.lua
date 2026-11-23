-- Keep frozen-map matching settings, but register only the rangefinder stream.
local lidar_options = include "android_2d_continue.lua"
TRAJECTORY_BUILDER_2D.use_imu_data = false
lidar_options.tracking_frame = "base_link"
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.0)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10.0
return lidar_options

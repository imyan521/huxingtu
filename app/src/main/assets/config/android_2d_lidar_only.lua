-- New KLD42 fallback: do not fuse the observed invalid angular-velocity stream.
local lidar_options = include "android_2d.lua"
TRAJECTORY_BUILDER_2D.use_imu_data = false
lidar_options.tracking_frame = "base_link"

-- The recorded 10 Hz scans rotate by up to about 10 degrees between frames.
-- Allow acceleration into turns without the former erroneous IMU prediction.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.0)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10.0
return lidar_options

-- Supplement-collection profile. Load the normal 2D configuration first, then
-- reduce the peak cost of global matching against a potentially large frozen
-- map. The old trajectories remain frozen and final optimization is unchanged.
local continue_options = include "android_2d.lua"

-- Coarser loop-closure points retain room/corridor structure while reducing
-- branch-and-bound work for every frozen-map candidate.
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.30
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 70

-- Do not test every new node against every old submap. A 35% global sample,
-- combined with the existing requirement for three independently matched
-- nodes, keeps relocalization reliable without saturating the phone.
POSE_GRAPH.global_sampling_ratio = 0.35
POSE_GRAPH.constraint_builder.sampling_ratio = 0.25
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 7
POSE_GRAPH.global_constraint_search_after_n_seconds = 0.5

-- Avoid repeated graph optimization while global constraint workers are still
-- locating the new trajectory. Final optimization still runs on FinishTrajectory.
POSE_GRAPH.optimize_every_n_nodes = 40

return continue_options

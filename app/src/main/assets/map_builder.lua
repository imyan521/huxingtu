include "pose_graph.lua"

MAP_BUILDER = {
  use_trajectory_builder_2d = false,
  use_trajectory_builder_3d = false,
  num_background_threads = 4,
  pose_graph = POSE_GRAPH,
  collate_by_trajectory = false, -- 👈 留着它，别让 C++ 找不到它
} -- 👈 🌟 这里的右括号绝对不能丢！
-- Copyright 2016 The Cartographer Authors
-- 必须保留 2D 和 3D 的基础定义，即使 3D 不运行
include "trajectory_builder_2d.lua"
include "trajectory_builder_3d.lua" -- 🌟 必须保留，不能干掉！

TRAJECTORY_BUILDER = {
  trajectory_builder_2d = TRAJECTORY_BUILDER_2D,
  trajectory_builder_3d = TRAJECTORY_BUILDER_3D, -- 🌟 必须保留占位

  -- 即使不喂数据，这两个 Key 也要留着给 C++ 读取，设为 false 即可
  collate_fixed_frame = false,
  collate_landmarks = false,
} -- 🌟 确保有闭合括号
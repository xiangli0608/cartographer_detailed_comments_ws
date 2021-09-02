-- Copyright 2016 The Cartographer Authors
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

POSE_GRAPH = {
  -- 每隔多少个节点执行一次后端优化
  optimize_every_n_nodes = 90,

  -- 约束构建的相关参数
  constraint_builder = {
    sampling_ratio = 0.3,                 -- 对局部子图进行回环检测时的计算频率, 数值越大, 计算次数越多
    max_constraint_distance = 15.,        -- 对局部子图进行回环检测时能成为约束的最大距离
    min_score = 0.55,                     -- 对局部子图进行回环检测时的最低分数阈值
    global_localization_min_score = 0.6,  -- 对整体子图进行回环检测时的最低分数阈值
    loop_closure_translation_weight = 1.1e4,
    loop_closure_rotation_weight = 1e5,
    log_matches = true,                   -- 打印约束计算的log
    
    -- 基于分支定界算法的2d粗匹配器
    fast_correlative_scan_matcher = {
      linear_search_window = 7.,
      angular_search_window = math.rad(30.),
      branch_and_bound_depth = 7,
    },

    -- 基于ceres的2d精匹配器
    ceres_scan_matcher = {
      occupied_space_weight = 20.,
      translation_weight = 10.,
      rotation_weight = 1.,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },

    -- 基于分支定界算法的3d粗匹配器
    fast_correlative_scan_matcher_3d = {
      branch_and_bound_depth = 8,
      full_resolution_depth = 3,
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 5.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(15.),
    },

    -- 基于ceres的3d精匹配器
    ceres_scan_matcher_3d = {
      occupied_space_weight_0 = 5.,
      occupied_space_weight_1 = 30.,
      translation_weight = 10.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
  },

  matcher_translation_weight = 5e2,
  matcher_rotation_weight = 1.6e3,

  -- 优化残差方程的相关参数
  optimization_problem = {
    huber_scale = 1e1,                -- 值越大,（潜在）异常值的影响就越大
    acceleration_weight = 1.1e2,      -- 3d里imu的线加速度的权重
    rotation_weight = 1.6e4,          -- 3d里imu的旋转的权重
    
    -- 前端结果残差的权重
    local_slam_pose_translation_weight = 1e5,
    local_slam_pose_rotation_weight = 1e5,
    -- 里程计残差的权重
    odometry_translation_weight = 1e5,
    odometry_rotation_weight = 1e5,
    -- gps残差的权重
    fixed_frame_pose_translation_weight = 1e1,
    fixed_frame_pose_rotation_weight = 1e2,
    fixed_frame_pose_use_tolerant_loss = false,
    fixed_frame_pose_tolerant_loss_param_a = 1,
    fixed_frame_pose_tolerant_loss_param_b = 1,

    log_solver_summary = false,
    use_online_imu_extrinsics_in_3d = true,
    fix_z_in_3d = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },

  max_num_final_iterations = 200,   -- 在建图结束之后执行一次全局优化, 不要求实时性, 迭代次数多
  global_sampling_ratio = 0.003,    -- 纯定位时候查找回环的频率
  log_residual_histograms = true,
  global_constraint_search_after_n_seconds = 10., -- 纯定位时多少秒执行一次全子图的约束计算

  --  overlapping_submaps_trimmer_2d = {
  --    fresh_submaps_count = 1,
  --    min_covered_area = 2,
  --    min_added_submaps_count = 5,
  --  },
}

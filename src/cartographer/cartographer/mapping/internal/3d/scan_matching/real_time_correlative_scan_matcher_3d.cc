/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/mapping/internal/3d/scan_matching/real_time_correlative_scan_matcher_3d.h"

#include <cmath>

#include "Eigen/Geometry"
#include "cartographer/common/math.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

RealTimeCorrelativeScanMatcher3D::RealTimeCorrelativeScanMatcher3D(
    const proto::RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {}

/**
 * @brief 通过暴力搜索方法对先验位姿进行校准
 * 
 * @param[in] initial_pose_estimate 先验位姿
 * @param[in] point_cloud 高分辨率点云
 * @param[in] hybrid_grid 高分辨率地图
 * @param[out] pose_estimate 校准后的位姿
 * @return float 
 */
float RealTimeCorrelativeScanMatcher3D::Match(
    const transform::Rigid3d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const HybridGrid& hybrid_grid,
    transform::Rigid3d* pose_estimate) const {
  CHECK(pose_estimate != nullptr);
  float best_score = -1.f;
  // 生成所有候选解，并遍历所有的候选解
  for (const transform::Rigid3f& transform : GenerateExhaustiveSearchTransforms(
           hybrid_grid.resolution(), point_cloud)) {
    // 将初始位姿乘以transform得到候选解的位姿
    const transform::Rigid3f candidate =
        initial_pose_estimate.cast<float>() * transform;
    // 为候选解打分
    const float score = ScoreCandidate(
        hybrid_grid, sensor::TransformPointCloud(point_cloud, candidate),
        transform);
    // 只保存最高分
    if (score > best_score) {
      best_score = score;
      *pose_estimate = candidate.cast<double>();
    }
  }
  return best_score;
}

// 生成所有可能解的位姿: xyz三个方向的平移与分别绕xyz的旋转
std::vector<transform::Rigid3f>
RealTimeCorrelativeScanMatcher3D::GenerateExhaustiveSearchTransforms(
    const float resolution, const sensor::PointCloud& point_cloud) const {
  std::vector<transform::Rigid3f> result;
  // xyz方向的搜索范围
  const int linear_window_size =
      common::RoundToInt(options_.linear_search_window() / resolution);
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  // 计算点云的点的最大距离
  float max_scan_range = 3.f * resolution;
  for (const sensor::RangefinderPoint& point : point_cloud) {
    const float range = point.position.norm();
    max_scan_range = std::max(range, max_scan_range);
  }
  const float kSafetyMargin = 1.f - 1e-3f;
  // 角度搜索步长
  const float angular_step_size =
      kSafetyMargin * std::acos(1.f - common::Pow2(resolution) /
                                          (2.f * common::Pow2(max_scan_range)));
  // 角度搜索范围
  const int angular_window_size =
      common::RoundToInt(options_.angular_search_window() / angular_step_size);
  // 生成所有可能的候选解
  for (int z = -linear_window_size; z <= linear_window_size; ++z) {
    for (int y = -linear_window_size; y <= linear_window_size; ++y) {
      for (int x = -linear_window_size; x <= linear_window_size; ++x) {
        for (int rz = -angular_window_size; rz <= angular_window_size; ++rz) {
          for (int ry = -angular_window_size; ry <= angular_window_size; ++ry) {
            for (int rx = -angular_window_size; rx <= angular_window_size;
                 ++rx) {
              const Eigen::Vector3f angle_axis(rx * angular_step_size,
                                               ry * angular_step_size,
                                               rz * angular_step_size);
              result.emplace_back(
                  Eigen::Vector3f(x * resolution, y * resolution,
                                  z * resolution),
                  transform::AngleAxisVectorToRotationQuaternion(angle_axis));
            }
          }
        }
      }
    }
  }
  return result;
}

// 对候选解进行打分
float RealTimeCorrelativeScanMatcher3D::ScoreCandidate(
    const HybridGrid& hybrid_grid,
    const sensor::PointCloud& transformed_point_cloud,
    const transform::Rigid3f& transform) const {
  float score = 0.f;
  // 点云的总得分
  for (const sensor::RangefinderPoint& point : transformed_point_cloud) {
    score +=
        hybrid_grid.GetProbability(hybrid_grid.GetCellIndex(point.position));
  }
  // 平均得分
  score /= static_cast<float>(transformed_point_cloud.size());
  const float angle = transform::GetAngle(transform);
  // 根据平移与旋转的权重对得分进行加权
  score *=
      std::exp(-common::Pow2(transform.translation().norm() *
                                 options_.translation_delta_cost_weight() +
                             angle * options_.rotation_delta_cost_weight()));
  CHECK_GT(score, 0.f);
  return score;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

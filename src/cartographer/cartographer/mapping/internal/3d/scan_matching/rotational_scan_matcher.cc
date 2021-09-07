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

#include "cartographer/mapping/internal/3d/scan_matching/rotational_scan_matcher.h"

#include <map>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

namespace {

constexpr float kMinDistance = 0.2f;
constexpr float kMaxDistance = 0.9f;
constexpr float kSliceHeight = 0.2f;

// 根据角度作为索引, 将value加入到直方图中进行累加
void AddValueToHistogram(float angle, const float value,
                         Eigen::VectorXf* histogram) {
  // Map the angle to [0, pi), i.e. a vector and its inverse are considered to
  // represent the same angle.
  // 将角度映射到 [0, pi), 即向量及其逆被视为表示相同的角度
  while (angle > static_cast<float>(M_PI)) {
    angle -= static_cast<float>(M_PI);
  }
  while (angle < 0.f) {
    angle += static_cast<float>(M_PI);
  }
  // [0, 1)
  const float zero_to_one = angle / static_cast<float>(M_PI);
  // 角度对应直方图的索引
  const int bucket = common::Clamp<int>(
      common::RoundToInt(histogram->size() * zero_to_one - 0.5f), 0,
      histogram->size() - 1);
  // 直方图元素即为权重的累加值
  (*histogram)(bucket) += value;
}

// 计算这帧点云坐标的平均值
Eigen::Vector3f ComputeCentroid(const sensor::PointCloud& slice) {
  CHECK(!slice.empty());
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  for (const sensor::RangefinderPoint& point : slice) {
    sum += point.position;
  }
  return sum / static_cast<float>(slice.size());
}

// 将切片后的点云加入到直方图中
void AddPointCloudSliceToHistogram(const sensor::PointCloud& slice,
                                   Eigen::VectorXf* const histogram) {
  if (slice.empty()) {
    return;
  }
  // We compute the angle of the ray from a point to the centroid of the whole
  // point cloud. If it is orthogonal to the angle we compute between points, we
  // will add the angle between points to the histogram with the maximum weight.
  // This is to reject, e.g., the angles observed on the ceiling and floor.
  const Eigen::Vector3f centroid = ComputeCentroid(slice);
  // 设置第一个点为参考点, 第一个点与x轴形成的角度最小
  Eigen::Vector3f last_point_position = slice.points().front().position;
  for (const sensor::RangefinderPoint& point : slice) {
    const Eigen::Vector2f delta =
        (point.position - last_point_position).head<2>();
    const Eigen::Vector2f direction = (point.position - centroid).head<2>();
    const float distance = delta.norm();
    if (distance < kMinDistance || direction.norm() < kMinDistance) {
      continue;
    }
    if (distance > kMaxDistance) {
      last_point_position = point.position;
      continue;
    }
    const float angle = common::atan2(delta);
    
    // 权重的计算思想，当ab和bc*乎垂直时, 说明ab可能是一个垂直于激光扫描线束的*面, 
    // 这时探测结果应该是比较准确的, 相反, 如果ab'如何过b‘c*乎*行时, 
    // 说明扫描线束乎行与障碍物, 这时可能探测误差较大（可能由传感器引起, 也有可能是孔洞、玻璃等）
    // 因此权重应该降低
    const float value = std::max(
        0.f, 1.f - std::abs(delta.normalized().dot(direction.normalized())));
    AddValueToHistogram(angle, value, histogram);
  }
}

// A function to sort the points in each slice by angle around the centroid.
// This is because the returns from different rangefinders are interleaved in
// the data.
// 将点云中的每个点与质心连线与x的夹角进行排序
sensor::PointCloud SortSlice(const sensor::PointCloud& slice) {
  struct SortableAnglePointPair {
    bool operator<(const SortableAnglePointPair& rhs) const {
      return angle < rhs.angle;
    }

    float angle;
    sensor::RangefinderPoint point;
  };
  // 计算这帧点云坐标的平均值
  const Eigen::Vector3f centroid = ComputeCentroid(slice);
  std::vector<SortableAnglePointPair> by_angle;
  by_angle.reserve(slice.size());
  for (const sensor::RangefinderPoint& point : slice) {
    const Eigen::Vector2f delta = (point.position - centroid).head<2>();
    if (delta.norm() < kMinDistance) {
      continue;
    }
    // 每个point与质心连线与x轴所成的角度
    by_angle.push_back(SortableAnglePointPair{common::atan2(delta), point});
  }
  // 夹角从小到大进行排序
  std::sort(by_angle.begin(), by_angle.end());
  sensor::PointCloud result;
  for (const auto& pair : by_angle) {
    result.push_back(pair.point);
  }
  return result;
}

// 计算两个直方图的余弦距离 即相似程度
float MatchHistograms(const Eigen::VectorXf& submap_histogram,
                      const Eigen::VectorXf& scan_histogram) {
  // We compute the dot product of normalized histograms as a measure of
  // similarity.
  // 我们计算归一化直方图的点积作为相似度的度量
  const float scan_histogram_norm = scan_histogram.norm();
  const float submap_histogram_norm = submap_histogram.norm();
  const float normalization = scan_histogram_norm * submap_histogram_norm;
  if (normalization < 1e-3f) {
    return 1.f;
  }
  // 返回两个直方图的夹角的cos值 a·b = |a|*|b|*cos<a,b>
  return submap_histogram.dot(scan_histogram) / normalization;
}

}  // namespace

RotationalScanMatcher::RotationalScanMatcher(const Eigen::VectorXf* histogram)
    : histogram_(histogram) {}

// Rotates the given 'histogram' by the given 'angle'. This might lead to
// rotations of a fractional bucket which is handled by linearly interpolating.
// 将给定的直方图旋转给定的角度
Eigen::VectorXf RotationalScanMatcher::RotateHistogram(
    const Eigen::VectorXf& histogram, const float angle) {
  if (histogram.size() == 0) {
    return histogram;
  }
  const float rotate_by_buckets = -angle * histogram.size() / M_PI;
  int full_buckets = common::RoundToInt(rotate_by_buckets - 0.5f);
  const float fraction = rotate_by_buckets - full_buckets;
  CHECK_GT(histogram.size(), 0);
  while (full_buckets < 0) {
    full_buckets += histogram.size();
  }
  Eigen::VectorXf rotated_histogram_0 = Eigen::VectorXf::Zero(histogram.size());
  Eigen::VectorXf rotated_histogram_1 = Eigen::VectorXf::Zero(histogram.size());
  for (int i = 0; i != histogram.size(); ++i) {
    rotated_histogram_0[i] = histogram[(i + full_buckets) % histogram.size()];
    rotated_histogram_1[i] =
        histogram[(i + 1 + full_buckets) % histogram.size()];
  }
  // 进行插值
  return fraction * rotated_histogram_1 +
         (1.f - fraction) * rotated_histogram_0;
}

// 计算点云的直方图
Eigen::VectorXf RotationalScanMatcher::ComputeHistogram(
    const sensor::PointCloud& point_cloud, const int histogram_size) {
  Eigen::VectorXf histogram = Eigen::VectorXf::Zero(histogram_size);
  // 将输入点云按照Z轴的高度切成n个切片
  std::map<int, sensor::PointCloud> slices;
  for (const sensor::RangefinderPoint& point : point_cloud) {
    slices[common::RoundToInt(point.position.z() / kSliceHeight)].push_back(
        point);
  }
  for (const auto& slice : slices) {
    // 将切片后的点云加入到直方图中
    AddPointCloudSliceToHistogram(SortSlice(slice.second), &histogram);
  }
  return histogram;
}

// 计算两个直方图的余弦距离
std::vector<float> RotationalScanMatcher::Match(
    const Eigen::VectorXf& histogram, const float initial_angle,
    const std::vector<float>& angles) const {
  std::vector<float> result;
  result.reserve(angles.size());
  for (const float angle : angles) {
    // 将直方图进行旋转
    const Eigen::VectorXf scan_histogram =
        RotateHistogram(histogram, initial_angle + angle);
    // 计算两个直方图向量的夹角cos值
    result.push_back(MatchHistograms(*histogram_, scan_histogram));
  }
  return result;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

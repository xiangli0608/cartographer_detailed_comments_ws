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

#include "cartographer/sensor/internal/voxel_filter.h"

#include <cmath>
#include <random>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "cartographer/common/math.h"

namespace cartographer {
namespace sensor {

namespace {

// 只有点小于最大距离时才进行拷贝
PointCloud FilterByMaxRange(const PointCloud& point_cloud,
                            const float max_range) {
  return point_cloud.copy_if([max_range](const RangefinderPoint& point) {
    return point.position.norm() <= max_range;
  });
}

// 自适应体素滤波
PointCloud AdaptivelyVoxelFiltered(
    const proto::AdaptiveVoxelFilterOptions& options,
    const PointCloud& point_cloud) {
  // param: adaptive_voxel_filter.min_num_points 满足小于等于这个值的点云满足要求, 足够稀疏
  if (point_cloud.size() <= options.min_num_points()) {
    // 'point_cloud' is already sparse enough.
    return point_cloud;
  }
  // param: adaptive_voxel_filter.max_length 进行一次体素滤波
  PointCloud result = VoxelFilter(point_cloud, options.max_length());
  // 如果按照最大边长进行体素滤波之后还超过这个数了, 就说明已经是这个参数下最稀疏的状态了, 直接返回
  if (result.size() >= options.min_num_points()) {
    // Filtering with 'max_length' resulted in a sufficiently dense point cloud.
    return result;
  }

  // Search for a 'low_length' that is known to result in a sufficiently
  // dense point cloud. We give up and use the full 'point_cloud' if reducing
  // the edge length by a factor of 1e-2 is not enough.
  // 将体素滤波的边长从max_length逐渐减小, 每次除以2
  for (float high_length = options.max_length();
       high_length > 1e-2f * options.max_length(); high_length /= 2.f) {
    // 减小边长再次进行体素滤波
    float low_length = high_length / 2.f;
    result = VoxelFilter(point_cloud, low_length);
    
    // 重复for循环直到 滤波后的点数多于min_num_points
    if (result.size() >= options.min_num_points()) {
      // Binary search to find the right amount of filtering. 'low_length' gave
      // a sufficiently dense 'result', 'high_length' did not. We stop when the
      // edge length is at most 10% off.
      // 以二分查找的方式找到合适的过滤边长, 当边缘长度最多减少 10% 时停止
      while ((high_length - low_length) / low_length > 1e-1f) {
        const float mid_length = (low_length + high_length) / 2.f;
        const PointCloud candidate = VoxelFilter(point_cloud, mid_length);
        // 如果点数多, 就将边长变大, 让low_length变大
        if (candidate.size() >= options.min_num_points()) {
          low_length = mid_length;
          result = candidate;
        } 
        // 如果点数少, 就将边长变小, 让high_length变小
        else {
          high_length = mid_length;
        }
      }
      return result;
    }
  }
  return result;
}

using VoxelKeyType = uint64_t;

// 计算该点处于的voxel的序号
VoxelKeyType GetVoxelCellIndex(const Eigen::Vector3f& point,
                               const float resolution) {
  const Eigen::Array3f index = point.array() / resolution;
  const uint64_t x = common::RoundToInt(index.x());
  const uint64_t y = common::RoundToInt(index.y());
  const uint64_t z = common::RoundToInt(index.z());
  return (x << 42) + (y << 21) + z;
}

// 进行体素滤波, 标记体素滤波后的点
template <class T, class PointFunction>
std::vector<bool> RandomizedVoxelFilterIndices(
    const std::vector<T>& point_cloud, const float resolution,
    PointFunction&& point_function) {
  // According to https://en.wikipedia.org/wiki/Reservoir_sampling
  std::minstd_rand0 generator;
  // std::pair<int, int>的第一个元素保存该voxel内部的点的个数, 第二个元素保存该voxel中选择的那一个点的序号
  absl::flat_hash_map<VoxelKeyType, std::pair<int, int>>
      voxel_count_and_point_index;
  // 遍历所有的点, 计算
  for (size_t i = 0; i < point_cloud.size(); i++) {
    // 获取VoxelKeyType对应的value的引用
    auto& voxel = voxel_count_and_point_index[GetVoxelCellIndex(
        point_function(point_cloud[i]), resolution)];
    voxel.first++;
    // 如果这个体素格子只有1个点, 那这个体素格子里的点的索引就是i
    if (voxel.first == 1) {
      voxel.second = i;
    } 
    else {
      // 生成随机数的范围是 [1, voxel.first]
      std::uniform_int_distribution<> distribution(1, voxel.first);
      // 生成的随机数与个数相等, 就让这个点代表这个体素格子
      if (distribution(generator) == voxel.first) {
        voxel.second = i;
      }
    }
  }

  // 为体素滤波之后的点做标记
  std::vector<bool> points_used(point_cloud.size(), false);
  for (const auto& voxel_and_index : voxel_count_and_point_index) {
    points_used[voxel_and_index.second.second] = true;
  }
  return points_used;
}

template <class T, class PointFunction>
std::vector<T> RandomizedVoxelFilter(const std::vector<T>& point_cloud,
                                     const float resolution,
                                     PointFunction&& point_function) {
  const std::vector<bool> points_used =
      RandomizedVoxelFilterIndices(point_cloud, resolution, point_function);

  std::vector<T> results;
  for (size_t i = 0; i < point_cloud.size(); i++) {
    if (points_used[i]) {
      results.push_back(point_cloud[i]);
    }
  }
  return results;
}

}  // namespace

std::vector<RangefinderPoint> VoxelFilter(
    const std::vector<RangefinderPoint>& points, const float resolution) {
  return RandomizedVoxelFilter(
      points, resolution,
      [](const RangefinderPoint& point) { return point.position; });
}

// 进行体素滤波
PointCloud VoxelFilter(const PointCloud& point_cloud, const float resolution) {
  // 得到标记后的点
  const std::vector<bool> points_used = RandomizedVoxelFilterIndices(
      point_cloud.points(), resolution,
      [](const RangefinderPoint& point) { return point.position; });

  // 生成滤波后的点云
  std::vector<RangefinderPoint> filtered_points;
  for (size_t i = 0; i < point_cloud.size(); i++) {
    if (points_used[i]) {
      filtered_points.push_back(point_cloud[i]);
    }
  }
  std::vector<float> filtered_intensities;
  CHECK_LE(point_cloud.intensities().size(), point_cloud.points().size());
  for (size_t i = 0; i < point_cloud.intensities().size(); i++) {
    if (points_used[i]) {
      filtered_intensities.push_back(point_cloud.intensities()[i]);
    }
  }

  return PointCloud(std::move(filtered_points),
                    std::move(filtered_intensities));
}

TimedPointCloud VoxelFilter(const TimedPointCloud& timed_point_cloud,
                            const float resolution) {
  return RandomizedVoxelFilter(
      timed_point_cloud, resolution,
      [](const TimedRangefinderPoint& point) { return point.position; });
}

std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement> VoxelFilter(
    const std::vector<sensor::TimedPointCloudOriginData::RangeMeasurement>&
        range_measurements,
    const float resolution) {
  return RandomizedVoxelFilter(
      range_measurements, resolution,
      [](const sensor::TimedPointCloudOriginData::RangeMeasurement&
             range_measurement) {
        return range_measurement.point_time.position;
      });
}

proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::AdaptiveVoxelFilterOptions options;
  options.set_max_length(parameter_dictionary->GetDouble("max_length"));
  options.set_min_num_points(
      parameter_dictionary->GetNonNegativeInt("min_num_points"));
  options.set_max_range(parameter_dictionary->GetDouble("max_range"));
  return options;
}

// 进行自适应体素滤波
PointCloud AdaptiveVoxelFilter(
    const PointCloud& point_cloud,
    const proto::AdaptiveVoxelFilterOptions& options) {
  return AdaptivelyVoxelFiltered(
      // param: adaptive_voxel_filter.max_range 距远离原点超过max_range的点被移除
      // 这里的最大距离是相对于local坐标系原点的
      options, FilterByMaxRange(point_cloud, options.max_range()));
}

}  // namespace sensor
}  // namespace cartographer

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

#include "cartographer/mapping/internal/2d/scan_matching/correlative_scan_matcher_2d.h"

#include <cmath>

#include "cartographer/common/math.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

// 构造函数
SearchParameters::SearchParameters(const double linear_search_window,
                                   const double angular_search_window,
                                   const sensor::PointCloud& point_cloud,
                                   const double resolution)
    : resolution(resolution) {
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  float max_scan_range = 3.f * resolution;

  // 求得 point_cloud 中雷达数据的 最大的值（最远点的距离）
  for (const sensor::RangefinderPoint& point : point_cloud) {
    const float range = point.position.head<2>().norm();
    max_scan_range = std::max(range, max_scan_range);
  }

  // 根据论文里的公式 求得角度分辨率 angular_perturbation_step_size
  const double kSafetyMargin = 1. - 1e-3;
  angular_perturbation_step_size =
      kSafetyMargin * std::acos(1. - common::Pow2(resolution) /
                                         (2. * common::Pow2(max_scan_range)));

  // 范围除以分辨率得到个数
  num_angular_perturbations =
      std::ceil(angular_search_window / angular_perturbation_step_size);
  // num_scans是要生成旋转点云的个数, 将 num_angular_perturbations 扩大了2倍
  num_scans = 2 * num_angular_perturbations + 1;

  // XY方向的搜索范围, 单位是多少个栅格
  const int num_linear_perturbations =
      std::ceil(linear_search_window / resolution);

  // linear_bounds 的作用是确定每一个点云的最大最小边界
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

// For testing.
SearchParameters::SearchParameters(const int num_linear_perturbations,
                                   const int num_angular_perturbations,
                                   const double angular_perturbation_step_size,
                                   const double resolution)
    : num_angular_perturbations(num_angular_perturbations),
      angular_perturbation_step_size(num_angular_perturbations),
      resolution(resolution),
      num_scans(2 * num_angular_perturbations + 1) {
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

// 计算每一帧点云 在保证最后一个点能在地图范围内时 的最大移动范围
void SearchParameters::ShrinkToFit(const std::vector<DiscreteScan2D>& scans,
                                   const CellLimits& cell_limits) {
  CHECK_EQ(scans.size(), num_scans);
  CHECK_EQ(linear_bounds.size(), num_scans);

  // 遍历生成的旋转后的很多scan
  for (int i = 0; i != num_scans; ++i) {
    Eigen::Array2i min_bound = Eigen::Array2i::Zero();
    Eigen::Array2i max_bound = Eigen::Array2i::Zero();

    // 对点云的每一个点进行遍历, 确定这帧点云的最大最小的坐标索引
    for (const Eigen::Array2i& xy_index : scans[i]) {
      // Array2i.min的作用是 获取对应元素的最小值组成新的Array2i
      min_bound = min_bound.min(-xy_index);
      max_bound = max_bound.max(Eigen::Array2i(cell_limits.num_x_cells - 1,
                                               cell_limits.num_y_cells - 1) -
                                xy_index);
    }

    // 计算每一帧点云 在保证最后一个点能在地图范围内时 的最大移动范围
    linear_bounds[i].min_x = std::max(linear_bounds[i].min_x, min_bound.x());
    linear_bounds[i].max_x = std::min(linear_bounds[i].max_x, max_bound.x());
    linear_bounds[i].min_y = std::max(linear_bounds[i].min_y, min_bound.y());
    linear_bounds[i].max_y = std::min(linear_bounds[i].max_y, max_bound.y());
  }
}

/*
void test_ShrinkToFit()
{
  std::vector<Eigen::Array2i> scan;
  scan.push_back(Eigen::Array2i(1, 1));
  scan.push_back(Eigen::Array2i(10, 1));
  scan.push_back(Eigen::Array2i(1, 10));
  scan.push_back(Eigen::Array2i(10, 10));

  Eigen::Array2i min_bound = Eigen::Array2i::Zero();
  Eigen::Array2i max_bound = Eigen::Array2i::Zero();
  int min_x = -140, max_x = 140, min_y = -140, max_y = 140;
  int num_x_cells = 100, num_y_cells = 100;

  // 对点云的每一个点进行遍历, 确定这帧点云的最大最小的坐标索引
  for (const Eigen::Array2i &xy_index : scan)
  {
    // Array2i.min的作用是 获取对应元素的最小值组成新的Array2i
    min_bound = min_bound.min(-xy_index);
    max_bound = max_bound.max(Eigen::Array2i(num_x_cells - 1,
                                             num_y_cells - 1) -
                              xy_index);
  }

  // 每一帧scan的最大最小的坐标索引
  min_x = std::max(min_x, min_bound.x());
  max_x = std::min(max_x, max_bound.x());
  min_y = std::max(min_y, min_bound.y());
  max_y = std::min(max_y, max_bound.y());
}
*/

// 生成按照不同角度旋转后的点云集合
std::vector<sensor::PointCloud> GenerateRotatedScans(
    const sensor::PointCloud& point_cloud,
    const SearchParameters& search_parameters) {
  std::vector<sensor::PointCloud> rotated_scans;
  // 生成 num_scans 个旋转后的点云
  rotated_scans.reserve(search_parameters.num_scans);
  // 起始角度
  double delta_theta = -search_parameters.num_angular_perturbations *
                       search_parameters.angular_perturbation_step_size;
  // 进行遍历，生成旋转不同角度后的点云集合
  for (int scan_index = 0; scan_index < search_parameters.num_scans;
       ++scan_index,
           delta_theta += search_parameters.angular_perturbation_step_size) {
    // 将 point_cloud 绕Z轴旋转了delta_theta
    rotated_scans.push_back(sensor::TransformPointCloud(
        point_cloud, transform::Rigid3f::Rotation(Eigen::AngleAxisf(
                         delta_theta, Eigen::Vector3f::UnitZ()))));
  }
  return rotated_scans;
}

// 将旋转后的点云集合按照预测出的平移量进行平移, 获取平移后的点在地图中的索引
std::vector<DiscreteScan2D> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<sensor::PointCloud>& scans,
    const Eigen::Translation2f& initial_translation) {
  // discrete_scans的size 为 旋转的点云的个数
  std::vector<DiscreteScan2D> discrete_scans;
  discrete_scans.reserve(scans.size());

  for (const sensor::PointCloud& scan : scans) {
    // discrete_scans中的每一个 DiscreteScan2D 的size设置为这一帧点云中所有点的个数
    discrete_scans.emplace_back();
    discrete_scans.back().reserve(scan.size());

    // 点云中的每一个点进行平移, 获取平移后的栅格索引
    for (const sensor::RangefinderPoint& point : scan) {
      // 对scan中的每个点进行平移
      const Eigen::Vector2f translated_point =
          Eigen::Affine2f(initial_translation) * point.position.head<2>();

      // 将旋转后的点 对应的栅格的索引放入discrete_scans
      discrete_scans.back().push_back(
          map_limits.GetCellIndex(translated_point));
    }
  }
  return discrete_scans;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

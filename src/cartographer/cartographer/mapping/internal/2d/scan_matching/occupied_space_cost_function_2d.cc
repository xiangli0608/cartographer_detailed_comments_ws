/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/mapping/internal/2d/scan_matching/occupied_space_cost_function_2d.h"

#include "cartographer/mapping/probability_values.h"
#include "ceres/cubic_interpolation.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

// Computes a cost for matching the 'point_cloud' to the 'grid' with
// a 'pose'. The cost increases with poorer correspondence of the grid and the
// point observation (e.g. points falling into less occupied space).
class OccupiedSpaceCostFunction2D {
 public:
  OccupiedSpaceCostFunction2D(const double scaling_factor,
                              const sensor::PointCloud& point_cloud,
                              const Grid2D& grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        grid_(grid) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    const GridArrayAdapter adapter(grid_);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const MapLimits& limits = grid_.limits();

    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].position.x())),
                                         (T(point_cloud_[i].position.y())),
                                         T(1.));
      // 根据预测位姿对单个点进行坐标变换
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      // 获取三次插值之后的栅格free值, Evaluate函数内部调用了GetValue函数
      interpolator.Evaluate(
          (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
              static_cast<double>(kPadding),
          &residual[i]);
      // free值越小, 表示占用的概率越大
      residual[i] = scaling_factor_ * residual[i];
    }
    return true;
  }

 private:
  static constexpr int kPadding = INT_MAX / 4;
  
  // 自定义网格
  class GridArrayAdapter {
   public:
    // 枚举 DATA_DIMENSION 表示被插值的向量或者函数的维度
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const Grid2D& grid) : grid_(grid) {}

    // 获取栅格free值
    void GetValue(const int row, const int column, double* const value) const {
      // 处于地图外部时, 赋予最大free值
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding) {
        *value = kMaxCorrespondenceCost;
      } 
      // 根据索引获取free值
      else {
        *value = static_cast<double>(grid_.GetCorrespondenceCost(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }

    // map上下左右各增加 kPadding
    int NumRows() const {
      return grid_.limits().cell_limits().num_y_cells + 2 * kPadding;
    }

    int NumCols() const {
      return grid_.limits().cell_limits().num_x_cells + 2 * kPadding;
    }

   private:
    const Grid2D& grid_;
  };

  OccupiedSpaceCostFunction2D(const OccupiedSpaceCostFunction2D&) = delete;
  OccupiedSpaceCostFunction2D& operator=(const OccupiedSpaceCostFunction2D&) =
      delete;

  const double scaling_factor_;
  const sensor::PointCloud& point_cloud_;
  const Grid2D& grid_;
};

}  // namespace

// 工厂函数, 返回地图的CostFunction
ceres::CostFunction* CreateOccupiedSpaceCostFunction2D(
    const double scaling_factor, const sensor::PointCloud& point_cloud,
    const Grid2D& grid) {
  return new ceres::AutoDiffCostFunction<OccupiedSpaceCostFunction2D,
                                         ceres::DYNAMIC /* residuals */,
                                         3 /* pose variables */>(
      new OccupiedSpaceCostFunction2D(scaling_factor, point_cloud, grid),
      point_cloud.size()); // 比固定残差维度的 多了一个参数
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

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

#ifndef CARTOGRAPHER_SENSOR_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_POINT_CLOUD_H_

#include <vector>

#include "Eigen/Core"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/sensor/rangefinder_point.h"
#include "cartographer/transform/rigid_transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

// Stores 3D positions of points together with some additional data, e.g.
// intensities.
/**
 * @brief 点云结构, 包含雷达一帧数据的所有数据点 与 数据点对应的强度值
 * 
 */
class PointCloud {
 public:
  using PointType = RangefinderPoint;

  PointCloud();
  explicit PointCloud(std::vector<PointType> points);
  PointCloud(std::vector<PointType> points, std::vector<float> intensities);

  // Returns the number of points in the point cloud.
  size_t size() const;
  // Checks whether there are any points in the point cloud.
  bool empty() const;

  const std::vector<PointType>& points() const;
  const std::vector<float>& intensities() const;
  const PointType& operator[](const size_t index) const;

  // Iterator over the points in the point cloud.
  using ConstIterator = std::vector<PointType>::const_iterator;
  ConstIterator begin() const;
  ConstIterator end() const;

  void push_back(PointType value);

  // Creates a PointCloud consisting of all the points for which `predicate`
  // returns true, together with the corresponding intensities.
  // 根据条件进行赋值
  template <class UnaryPredicate>
  PointCloud copy_if(UnaryPredicate predicate) const {
    std::vector<PointType> points;
    std::vector<float> intensities;

    // Note: benchmarks show that it is better to have this conditional outside
    // the loop.
    if (intensities_.empty()) {
      for (size_t index = 0; index < size(); ++index) {
        const PointType& point = points_[index];
        // 表达式为true时才使用这个点
        if (predicate(point)) {
          points.push_back(point);
        }
      }
    } else {
      for (size_t index = 0; index < size(); ++index) {
        const PointType& point = points_[index];
        if (predicate(point)) {
          points.push_back(point);
          intensities.push_back(intensities_[index]);
        }
      }
    }

    return PointCloud(points, intensities);
  }

 private:
  // For 2D points, the third entry is 0.f.
  std::vector<PointType> points_;
  // Intensities are optional. If non-empty, they must have the same size as
  // points.
  std::vector<float> intensities_;
};

// Stores 3D positions of points with their relative measurement time in the
// fourth entry. Time is in seconds, increasing and relative to the moment when
// the last point was acquired. So, the fourth entry for the last point is 0.f.
// If timing is not available, all fourth entries are 0.f. For 2D points, the
// third entry is 0.f (and the fourth entry is time).
// 将点的3D位置及其相对测量时间存储在第四项中.
// 时间以秒为单位, 相对于获取最后一点的时间增加. 因此, 最后一点的第四个条目是0.f.
// 如果计时不可用, 则所有第四项均为0.f. 对于2D点, z坐标是0.f（第四项是时间）.
using TimedPointCloud = std::vector<TimedRangefinderPoint>;

// TODO(wohe): Retained for cartographer_ros. To be removed once it is no
// longer used there.
struct PointCloudWithIntensities {
  TimedPointCloud points;
  std::vector<float> intensities;
};

// Transforms 'point_cloud' according to 'transform'.
PointCloud TransformPointCloud(const PointCloud& point_cloud,
                               const transform::Rigid3f& transform);

// Transforms 'point_cloud' according to 'transform'.
TimedPointCloud TransformTimedPointCloud(const TimedPointCloud& point_cloud,
                                         const transform::Rigid3f& transform);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
PointCloud CropPointCloud(const PointCloud& point_cloud, float min_z,
                          float max_z);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_POINT_CLOUD_H_

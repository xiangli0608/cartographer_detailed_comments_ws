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

#include "cartographer/sensor/range_data.h"

#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {

// 根据给定的坐标变换, 分别对origin, returns, misses做变换, 拷贝
RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform) {
  return RangeData{
      transform * range_data.origin,
      TransformPointCloud(range_data.returns, transform), // 拷贝
      TransformPointCloud(range_data.misses, transform),  // 拷贝
  };
}

/**
 * @brief 对输入的点云进行z轴上的过滤
 * 
 * @param[in] range_data 原始点云数据
 * @param[in] min_z 最小的z坐标
 * @param[in] max_z 最大的z坐标
 * @return RangeData 裁剪之后的点云 拷贝
 */
RangeData CropRangeData(const RangeData& range_data, const float min_z,
                        const float max_z) {
  return RangeData{range_data.origin,
                   CropPointCloud(range_data.returns, min_z, max_z),  // 拷贝
                   CropPointCloud(range_data.misses, min_z, max_z)};  // 拷贝
}

proto::RangeData ToProto(const RangeData& range_data) {
  proto::RangeData proto;
  *proto.mutable_origin() = transform::ToProto(range_data.origin);
  proto.mutable_returns()->Reserve(range_data.returns.size());
  for (const RangefinderPoint& point : range_data.returns) {
    *proto.add_returns() = ToProto(point);
  }
  proto.mutable_misses()->Reserve(range_data.misses.size());
  for (const RangefinderPoint& point : range_data.misses) {
    *proto.add_misses() = ToProto(point);
  }
  return proto;
}

RangeData FromProto(const proto::RangeData& proto) {
  std::vector<RangefinderPoint> returns;
  if (proto.returns_size() > 0) {
    returns.reserve(proto.returns().size());
    for (const auto& point_proto : proto.returns()) {
      returns.push_back(FromProto(point_proto));
    }
  } else {
    returns.reserve(proto.returns_legacy().size());
    for (const auto& point_proto : proto.returns_legacy()) {
      returns.push_back({transform::ToEigen(point_proto)});
    }
  }
  std::vector<RangefinderPoint> misses;
  if (proto.misses_size() > 0) {
    misses.reserve(proto.misses().size());
    for (const auto& point_proto : proto.misses()) {
      misses.push_back(FromProto(point_proto));
    }
  } else {
    misses.reserve(proto.misses_legacy().size());
    for (const auto& point_proto : proto.misses_legacy()) {
      misses.push_back({transform::ToEigen(point_proto)});
    }
  }
  return RangeData{transform::ToEigen(proto.origin()), PointCloud(returns),
                   PointCloud(misses)};
}

}  // namespace sensor
}  // namespace cartographer

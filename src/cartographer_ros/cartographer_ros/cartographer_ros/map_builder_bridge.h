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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H

#include <memory>
#include <set>
#include <string>
#include <unordered_map>

#include "absl/synchronization/mutex.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/trajectory_builder_options.pb.h"
#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/mapping/id.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/SubmapEntry.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "cartographer_ros_msgs/TrajectoryQuery.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/OccupancyGrid.h"

// Abseil unfortunately pulls in winnt.h, which #defines DELETE.
// Clean up to unbreak visualization_msgs::Marker::DELETE.
#ifdef DELETE
#undef DELETE
#endif
#include "visualization_msgs/MarkerArray.h"

namespace cartographer_ros {

// lx add
using ::cartographer::mapping::NodeId;
using ::cartographer::mapping::MapById;
using ::cartographer::mapping::TrajectoryNode;

class MapBuilderBridge {
 public:

/**
 * note: local frame 与 global frame
 * carographer中存在两个地图坐标系, 分别为global frame与local frame
 * 
 * local frame
 * 是表达local slam结果的坐标系, 是固定的坐标系, 不会被回环检测与位姿图优化所更改, 
 * 其每一帧位姿间的坐标变换不会改变
 * 
 * global frame
 * 是表达被回环检测与位姿图优化所更改后的坐标系, 当有新的优化结果可用时, 此坐标系与任何其他坐标系之间的转换都会跳变.
 * 它的z轴指向上方, 即重力加速度矢量指向-z方向, 即由加速度计测得的重力分量沿+z方向.
 */

  struct LocalTrajectoryData {
    // Contains the trajectory data received from local SLAM, after
    // it had processed accumulated 'range_data_in_local' and estimated
    // current 'local_pose' at 'time'.
    
    // LocalSlamData中包含了local slam的一些数据, 包含当前时间, 当前估计的位姿, 以及累计的所有雷达数据
    struct LocalSlamData {
      ::cartographer::common::Time time;
      ::cartographer::transform::Rigid3d local_pose;
      ::cartographer::sensor::RangeData range_data_in_local;
    };
    std::shared_ptr<const LocalSlamData> local_slam_data;
    cartographer::transform::Rigid3d local_to_map;  // local frame 到 global frame间的坐标变换

    // published_frame 到 tracking_frame 间的坐标变换
    std::unique_ptr<cartographer::transform::Rigid3d> published_to_tracking;
    TrajectoryOptions trajectory_options;

    // c++11: std::shared_ptr 主要的用途就是方便资源的管理, 自动释放没有指针引用的资源
    // 使用引用计数来标识是否有其余指针指向该资源.(注意, shart_ptr本身指针会占1个引用)
    // 引用计数是分配在动态分配的, std::shared_ptr支持拷贝, 新的指针获可以获取前引用计数个数
  };

  MapBuilderBridge(
      const NodeOptions& node_options,
      std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
      tf2_ros::Buffer* tf_buffer);

  MapBuilderBridge(const MapBuilderBridge&) = delete;
  MapBuilderBridge& operator=(const MapBuilderBridge&) = delete;

  void LoadState(const std::string& state_filename, bool load_frozen_state);
  int AddTrajectory(
      const std::set<
          ::cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& trajectory_options);
  void FinishTrajectory(int trajectory_id);
  void RunFinalOptimization();
  bool SerializeState(const std::string& filename,
                      const bool include_unfinished_submaps);

  void HandleSubmapQuery(
      cartographer_ros_msgs::SubmapQuery::Request& request,
      cartographer_ros_msgs::SubmapQuery::Response& response);
  void HandleTrajectoryQuery(
      cartographer_ros_msgs::TrajectoryQuery::Request& request,
      cartographer_ros_msgs::TrajectoryQuery::Response& response);

  std::map<int /* trajectory_id */,
           ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
  GetTrajectoryStates();
  cartographer_ros_msgs::SubmapList GetSubmapList();
  std::unordered_map<int, LocalTrajectoryData> GetLocalTrajectoryData()
      LOCKS_EXCLUDED(mutex_);
  visualization_msgs::MarkerArray GetTrajectoryNodeList();
  visualization_msgs::MarkerArray GetLandmarkPosesList();
  visualization_msgs::MarkerArray GetConstraintList();

  // lx add
  std::shared_ptr<MapById<NodeId, TrajectoryNode>> GetTrajectoryNodes();

  SensorBridge* sensor_bridge(int trajectory_id);

 private:
  void OnLocalSlamResult(const int trajectory_id,
                         const ::cartographer::common::Time time,
                         const ::cartographer::transform::Rigid3d local_pose,
                         ::cartographer::sensor::RangeData range_data_in_local)
      LOCKS_EXCLUDED(mutex_);

  absl::Mutex mutex_;
  const NodeOptions node_options_;
  std::unordered_map<int,
                     std::shared_ptr<const LocalTrajectoryData::LocalSlamData>>
      local_slam_data_ GUARDED_BY(mutex_);
  std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder_;
  tf2_ros::Buffer* const tf_buffer_;

  std::unordered_map<std::string /* landmark ID */, int> landmark_to_index_;

  // These are keyed with 'trajectory_id'.
  std::unordered_map<int, TrajectoryOptions> trajectory_options_;
  std::unordered_map<int, std::unique_ptr<SensorBridge>> sensor_bridges_;
  std::unordered_map<int, size_t> trajectory_to_highest_marker_id_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_MAP_BUILDER_BRIDGE_H

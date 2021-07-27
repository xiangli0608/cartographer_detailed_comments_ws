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

#include <cmath>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/synchronization/mutex.h"

// cairo是一个非常流行的开源2D图形渲染引擎库
#include "cairo/cairo.h"

#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "gflags/gflags.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

// 这里的分辨率只是生成地图的分辨率, 与实际cartographer使用的地图的分辨率无关
DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");
DEFINE_bool(include_frozen_submaps, true,
            "Include frozen submaps in the occupancy grid.");
DEFINE_bool(include_unfrozen_submaps, true,
            "Include unfrozen submaps in the occupancy grid.");
DEFINE_string(occupancy_grid_topic, cartographer_ros::kOccupancyGridTopic,
              "Name of the topic on which the occupancy grid is published.");

namespace cartographer_ros {
namespace {

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class Node {
 public:
  explicit Node(double resolution, double publish_period_sec);
  ~Node() {}

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

 private:
  void HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
  void DrawAndPublish(const ::ros::WallTimerEvent& timer_event);

  ::ros::NodeHandle node_handle_;
  const double resolution_;

  absl::Mutex mutex_;
  ::ros::ServiceClient client_ GUARDED_BY(mutex_);
  ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
  ::ros::Publisher occupancy_grid_publisher_ GUARDED_BY(mutex_);
  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  ::ros::WallTimer occupancy_grid_publisher_timer_;
  std::string last_frame_id_;
  ros::Time last_timestamp_;
};

Node::Node(const double resolution, const double publish_period_sec)
    : resolution_(resolution),
      // SubmapQuery服务的客户端
      client_(node_handle_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
          kSubmapQueryServiceName)),
      // 订阅 submap_list topic, 注册回调函数
      submap_list_subscriber_(node_handle_.subscribe(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize,
          boost::function<void(
              const cartographer_ros_msgs::SubmapList::ConstPtr&)>(
              [this](const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
                HandleSubmapList(msg);
              }))),
      // 声明 map 发布器
      occupancy_grid_publisher_(
          node_handle_.advertise<::nav_msgs::OccupancyGrid>(
              FLAGS_occupancy_grid_topic, kLatestOnlyPublisherQueueSize,
              true /* latched */)),
      // 定时发布map
      occupancy_grid_publisher_timer_(
          node_handle_.createWallTimer(::ros::WallDuration(publish_period_sec),
                                       &Node::DrawAndPublish, this)) {}

// 订阅SubmapList话题的回调函数
void Node::HandleSubmapList(
    const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  absl::MutexLock locker(&mutex_);

  // We do not do any work if nobody listens.
  // 如果没人订阅就不处理
  if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
    return;
  }

  // Keep track of submap IDs that don't appear in the message anymore.
  // 先将所有的submap_slices_保存到submap_ids_to_delete中
  std::set<SubmapId> submap_ids_to_delete;
  for (const auto& pair : submap_slices_) {
    submap_ids_to_delete.insert(pair.first);
  }

  // 遍历所有的submap
  for (const auto& submap_msg : msg->submap) {
    // 生成SubmapId
    const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
    // 如果msg中存在这个生成SubmapId就在 待删除列表 中移除
    submap_ids_to_delete.erase(id);
    // 根据参数决定 是否 对 frozen 与 unfrozen 的submap进行跳过
    if ((submap_msg.is_frozen && !FLAGS_include_frozen_submaps) ||
        (!submap_msg.is_frozen && !FLAGS_include_unfrozen_submaps)) {
      continue;
    }

    // 获取key为id的值的引用进行赋值, 如果key中没有这个id就新建一个键值对
    SubmapSlice& submap_slice = submap_slices_[id];
    submap_slice.pose = ToRigid3d(submap_msg.pose);
    submap_slice.metadata_version = submap_msg.submap_version;

    // 如果已经填充过地图信息了就跳过
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_msg.submap_version) {
      continue;
    }

    // Step: 1 获取格式为io::SubmapTextures的地图栅格数据
    auto fetched_textures =
        ::cartographer_ros::FetchSubmapTextures(id, &client_);

    if (fetched_textures == nullptr) {
      continue;
    }
    CHECK(!fetched_textures->textures.empty());
    submap_slice.version = fetched_textures->version;

    // We use the first texture only. By convention this is the highest
    // resolution texture and that is the one we want to use to construct the
    // map for ROS.
    // 将 格式为io::SubmapTextures的地图栅格数据 放入 submap_slice 中
    const auto fetched_texture = fetched_textures->textures.begin();
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;
    submap_slice.cairo_data.clear();
    
    // Step: 2 生成surface, surface是指向Cairo画布的指针
    submap_slice.surface = ::cartographer::io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
        fetched_texture->width, fetched_texture->height,
        &submap_slice.cairo_data);
  } // end for

  // Delete all submaps that didn't appear in the message.
  // msg处理完之后, 还在 待删除 列表中的信息 就是后来不出现的, 可以删掉了
  for (const auto& id : submap_ids_to_delete) {
    submap_slices_.erase(id);
  }

  last_timestamp_ = msg->header.stamp;
  last_frame_id_ = msg->header.frame_id;
}

// 时间驱动的函数
void Node::DrawAndPublish(const ::ros::WallTimerEvent& unused_timer_event) {
  absl::MutexLock locker(&mutex_);
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    return;
  }
  //  Step: 3 生成PaintSubmapSlicesResult, 绘制图像
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  
  // Step: 4 由cartographer格式的地图生成ros格式的地图
  std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_);
  
  //  Step: 5 发布map topic
  occupancy_grid_publisher_.publish(*msg_ptr);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(FLAGS_include_frozen_submaps || FLAGS_include_unfrozen_submaps)
      << "Ignoring both frozen and unfrozen submaps makes no sense.";

  ::ros::init(argc, argv, "cartographer_occupancy_grid_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  // 这个Node类指的是上边的类,与node.cc无关
  ::cartographer_ros::Node node(FLAGS_resolution, FLAGS_publish_period_sec);

  ::ros::spin();
  ::ros::shutdown();
}

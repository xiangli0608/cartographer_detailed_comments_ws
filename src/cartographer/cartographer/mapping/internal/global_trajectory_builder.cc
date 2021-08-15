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

#include "cartographer/mapping/internal/global_trajectory_builder.h"

#include <memory>

#include "absl/memory/memory.h"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/internal/local_slam_result_data.h"
#include "cartographer/mapping/internal/motion_filter.h"
#include "cartographer/metrics/family_factory.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

static auto* kLocalSlamMatchingResults = metrics::Counter::Null();
static auto* kLocalSlamInsertionResults = metrics::Counter::Null();

template <typename LocalTrajectoryBuilder, typename PoseGraph>
class GlobalTrajectoryBuilder : public mapping::TrajectoryBuilderInterface {
 public:
  // Passing a 'nullptr' for 'local_trajectory_builder' is acceptable, but no
  // 'TimedPointCloudData' may be added in that case.

  /**
   * @brief 完整的slam, 连接起了前端与后端
   * 
   * @param[in] local_trajectory_builder 2d or 3d local slam 前端
   * @param[in] trajectory_id 轨迹id
   * @param[in] pose_graph 2d or 3d pose_graph 后端
   * @param[in] local_slam_result_callback 前端的回调函数
   * @param[in] pose_graph_odometry_motion_filter 里程计的滤波器
   */
  GlobalTrajectoryBuilder(
      std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder,
      const int trajectory_id, PoseGraph* const pose_graph,
      const LocalSlamResultCallback& local_slam_result_callback,
      const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter)
      : trajectory_id_(trajectory_id),
        pose_graph_(pose_graph),
        local_trajectory_builder_(std::move(local_trajectory_builder)),
        local_slam_result_callback_(local_slam_result_callback),
        pose_graph_odometry_motion_filter_(pose_graph_odometry_motion_filter) {}
  ~GlobalTrajectoryBuilder() override {}

  // 禁用拷贝构造函数与赋值运算符
  GlobalTrajectoryBuilder(const GlobalTrajectoryBuilder&) = delete;
  GlobalTrajectoryBuilder& operator=(const GlobalTrajectoryBuilder&) = delete;

  /**
   * @brief 点云数据的处理, 先进行扫描匹配, 然后将扫描匹配的结果当做节点插入到后端的位姿图中
   * 
   * @param[in] sensor_id topic名字
   * @param[in] timed_point_cloud_data 点云数据
   */
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    CHECK(local_trajectory_builder_)
        << "Cannot add TimedPointCloudData without a LocalTrajectoryBuilder.";

    // 进行扫描匹配, 返回匹配后的结果
    std::unique_ptr<typename LocalTrajectoryBuilder::MatchingResult>
        matching_result = local_trajectory_builder_->AddRangeData(
            sensor_id, timed_point_cloud_data);

    if (matching_result == nullptr) {
      // The range data has not been fully accumulated yet.
      return;
    }
    kLocalSlamMatchingResults->Increment();
    std::unique_ptr<InsertionResult> insertion_result;

    // matching_result->insertion_result 的类型是 LocalTrajectoryBuilder2D::InsertionResult
    // 如果雷达成功插入到地图中
    if (matching_result->insertion_result != nullptr) {
      kLocalSlamInsertionResults->Increment();

      // 将匹配后的结果 当做节点 加入到位姿图中
      auto node_id = pose_graph_->AddNode(
          matching_result->insertion_result->constant_data, trajectory_id_,
          matching_result->insertion_result->insertion_submaps);
          
      CHECK_EQ(node_id.trajectory_id, trajectory_id_);

      // 这里的InsertionResult的类型是 TrajectoryBuilderInterface::InsertionResult
      insertion_result = absl::make_unique<InsertionResult>(InsertionResult{
          node_id, 
          matching_result->insertion_result->constant_data,
          std::vector<std::shared_ptr<const Submap>>(
              matching_result->insertion_result->insertion_submaps.begin(),
              matching_result->insertion_result->insertion_submaps.end())});
    }

    // 将结果数据传入回调函数中, 进行保存
    if (local_slam_result_callback_) {
      local_slam_result_callback_(
          trajectory_id_, matching_result->time, matching_result->local_pose,
          std::move(matching_result->range_data_in_local),
          std::move(insertion_result));
    }

  }

  // imu数据的处理, 数据走向有两个,一个是进入前端local_trajectory_builder_,一个是进入后端pose_graph_
  void AddSensorData(const std::string& sensor_id,
                     const sensor::ImuData& imu_data) override {
    if (local_trajectory_builder_) {
      local_trajectory_builder_->AddImuData(imu_data);
    }
    pose_graph_->AddImuData(trajectory_id_, imu_data);
  }

  // 里程计数据的处理, 数据走向有两个,一个是进入前端local_trajectory_builder_, 一个是进入后端pose_graph_
  // 加入到后端之前, 先做一个距离的计算, 只有时间,移动距离,角度 变换量大于阈值才加入到后端中
  void AddSensorData(const std::string& sensor_id,
                     const sensor::OdometryData& odometry_data) override {
    CHECK(odometry_data.pose.IsValid()) << odometry_data.pose;
    if (local_trajectory_builder_) {
      local_trajectory_builder_->AddOdometryData(odometry_data);
    }
    // TODO(MichaelGrupp): Instead of having an optional filter on this level,
    // odometry could be marginalized between nodes in the pose graph.
    // Related issue: cartographer-project/cartographer/#1768
    if (pose_graph_odometry_motion_filter_.has_value() &&
        pose_graph_odometry_motion_filter_.value().IsSimilar(
            odometry_data.time, odometry_data.pose)) {
      return;
    }
    pose_graph_->AddOdometryData(trajectory_id_, odometry_data);
  }

  // gps数据只在后端中使用
  void AddSensorData(
      const std::string& sensor_id,
      const sensor::FixedFramePoseData& fixed_frame_pose) override {
    if (fixed_frame_pose.pose.has_value()) {
      CHECK(fixed_frame_pose.pose.value().IsValid())
          << fixed_frame_pose.pose.value();
    }
    pose_graph_->AddFixedFramePoseData(trajectory_id_, fixed_frame_pose);
  }

  // Landmark的数据只在后端中使用
  void AddSensorData(const std::string& sensor_id,
                     const sensor::LandmarkData& landmark_data) override {
    pose_graph_->AddLandmarkData(trajectory_id_, landmark_data);
  }

  // 将local slam的结果加入到后端中, 作为位姿图的一个节点
  void AddLocalSlamResultData(std::unique_ptr<mapping::LocalSlamResultData>
                                  local_slam_result_data) override {
    CHECK(!local_trajectory_builder_) << "Can't add LocalSlamResultData with "
                                         "local_trajectory_builder_ present.";
    local_slam_result_data->AddToPoseGraph(trajectory_id_, pose_graph_);
  }

 private:
  const int trajectory_id_;
  PoseGraph* const pose_graph_;     // 模板参数, 可以指向PoseGraph2D也可以指向PoseGraph3D
  std::unique_ptr<LocalTrajectoryBuilder> local_trajectory_builder_;  // 模板参数
  LocalSlamResultCallback local_slam_result_callback_;
  absl::optional<MotionFilter> pose_graph_odometry_motion_filter_;
};

}  // namespace

// 类模板不支持实参推演, 所以在类外指定模板参数的具体类型, 再进行类的实例化

// 2d的完整的slam
std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder2D(
    std::unique_ptr<LocalTrajectoryBuilder2D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph2D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback,
    const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter) {
  return absl::make_unique<
      GlobalTrajectoryBuilder<LocalTrajectoryBuilder2D, mapping::PoseGraph2D>>(
      std::move(local_trajectory_builder), trajectory_id, pose_graph,
      local_slam_result_callback, pose_graph_odometry_motion_filter);
}

// 3d的完整的slam
std::unique_ptr<TrajectoryBuilderInterface> CreateGlobalTrajectoryBuilder3D(
    std::unique_ptr<LocalTrajectoryBuilder3D> local_trajectory_builder,
    const int trajectory_id, mapping::PoseGraph3D* const pose_graph,
    const TrajectoryBuilderInterface::LocalSlamResultCallback&
        local_slam_result_callback,
    const absl::optional<MotionFilter>& pose_graph_odometry_motion_filter) {
  return absl::make_unique<
      GlobalTrajectoryBuilder<LocalTrajectoryBuilder3D, mapping::PoseGraph3D>>(
      std::move(local_trajectory_builder), trajectory_id, pose_graph,
      local_slam_result_callback, pose_graph_odometry_motion_filter);
}

void GlobalTrajectoryBuilderRegisterMetrics(metrics::FamilyFactory* factory) {
  auto* results = factory->NewCounterFamily(
      "mapping_global_trajectory_builder_local_slam_results",
      "Local SLAM results");
  kLocalSlamMatchingResults = results->Add({{"type", "MatchingResult"}});
  kLocalSlamInsertionResults = results->Add({{"type", "InsertionResult"}});
}

}  // namespace mapping
}  // namespace cartographer

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

#include "cartographer/mapping/internal/optimization/optimization_problem_2d.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/internal/ceres_solver_options.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/optimization/ceres_pose.h"
#include "cartographer/mapping/internal/optimization/cost_functions/landmark_cost_function_2d.h"
#include "cartographer/mapping/internal/optimization/cost_functions/spa_cost_function_2d.h"
#include "cartographer/sensor/odometry_data.h"
#include "cartographer/transform/transform.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace optimization {
namespace {

using ::cartographer::mapping::optimization::CeresPose;
using LandmarkNode = ::cartographer::mapping::PoseGraphInterface::LandmarkNode;
using TrajectoryData =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryData;

// For fixed frame pose.
std::unique_ptr<transform::Rigid3d> Interpolate(
    const sensor::MapByTime<sensor::FixedFramePoseData>& map_by_time,
    const int trajectory_id, const common::Time time) {
  const auto it = map_by_time.lower_bound(trajectory_id, time);
  if (it == map_by_time.EndOfTrajectory(trajectory_id) ||
      !it->pose.has_value()) {
    return nullptr;
  }
  if (it == map_by_time.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return absl::make_unique<transform::Rigid3d>(it->pose.value());
    }
    return nullptr;
  }
  const auto prev_it = std::prev(it);
  if (prev_it->pose.has_value()) {
    return absl::make_unique<transform::Rigid3d>(
        Interpolate(transform::TimestampedTransform{prev_it->time,
                                                    prev_it->pose.value()},
                    transform::TimestampedTransform{it->time, it->pose.value()},
                    time)
            .transform);
  }
  return nullptr;
}

// Converts a pose into the 3 optimization variable format used for Ceres:
// translation in x and y, followed by the rotation angle representing the
// orientation.
// 将姿势转换为用于 Ceres 的 3 优化变量格式
// x 和 y 的平移, 然后是表示方向的旋转角度.
std::array<double, 3> FromPose(const transform::Rigid2d& pose) {
  return {{pose.translation().x(), pose.translation().y(),
           pose.normalized_angle()}};
}

// Converts a pose as represented for Ceres back to an transform::Rigid2d pose.
// 将 Ceres 表示的姿势转换回 transform::Rigid2d 姿势
transform::Rigid2d ToPose(const std::array<double, 3>& values) {
  return transform::Rigid2d({values[0], values[1]}, values[2]);
}

// Selects a trajectory node closest in time to the landmark observation and
// applies a relative transform from it.
transform::Rigid3d GetInitialLandmarkPose(
    const LandmarkNode::LandmarkObservation& observation,
    const NodeSpec2D& prev_node, const NodeSpec2D& next_node,
    const std::array<double, 3>& prev_node_pose,
    const std::array<double, 3>& next_node_pose) {
  const double interpolation_parameter =
      common::ToSeconds(observation.time - prev_node.time) /
      common::ToSeconds(next_node.time - prev_node.time);

  const std::tuple<std::array<double, 4>, std::array<double, 3>>
      rotation_and_translation =
          InterpolateNodes2D(prev_node_pose.data(), prev_node.gravity_alignment,
                             next_node_pose.data(), next_node.gravity_alignment,
                             interpolation_parameter);
  return transform::Rigid3d::FromArrays(std::get<0>(rotation_and_translation),
                                        std::get<1>(rotation_and_translation)) *
         observation.landmark_to_tracking_transform;
}

/**
 * @brief 
 * 
 * @param[in] landmark_nodes 
 * @param[in] node_data 
 * @param[in] C_nodes 
 * @param[in] C_landmarks 
 * @param[in] problem 
 * @param[in] huber_scale 
 */
void AddLandmarkCostFunctions(
    const std::map<std::string, LandmarkNode>& landmark_nodes,
    const MapById<NodeId, NodeSpec2D>& node_data,
    MapById<NodeId, std::array<double, 3>>* C_nodes,
    std::map<std::string, CeresPose>* C_landmarks, ceres::Problem* problem,
    double huber_scale) {
  for (const auto& landmark_node : landmark_nodes) {
    for (const auto& observation : landmark_node.second.landmark_observations) {
      const std::string& landmark_id = landmark_node.first;
      const auto& begin_of_trajectory =
          node_data.BeginOfTrajectory(observation.trajectory_id);
      // The landmark observation was made before the trajectory was created.
      if (observation.time < begin_of_trajectory->data.time) {
        continue;
      }
      // Find the trajectory nodes before and after the landmark observation.
      auto next =
          node_data.lower_bound(observation.trajectory_id, observation.time);
      // The landmark observation was made, but the next trajectory node has
      // not been added yet.
      if (next == node_data.EndOfTrajectory(observation.trajectory_id)) {
        continue;
      }
      if (next == begin_of_trajectory) {
        next = std::next(next);
      }
      auto prev = std::prev(next);
      // Add parameter blocks for the landmark ID if they were not added before.
      std::array<double, 3>* prev_node_pose = &C_nodes->at(prev->id);
      std::array<double, 3>* next_node_pose = &C_nodes->at(next->id);
      if (!C_landmarks->count(landmark_id)) {
        const transform::Rigid3d starting_point =
            landmark_node.second.global_landmark_pose.has_value()
                ? landmark_node.second.global_landmark_pose.value()
                : GetInitialLandmarkPose(observation, prev->data, next->data,
                                         *prev_node_pose, *next_node_pose);
        C_landmarks->emplace(
            landmark_id,
            CeresPose(starting_point, nullptr /* translation_parametrization */,
                      absl::make_unique<ceres::QuaternionParameterization>(),
                      problem));
        // Set landmark constant if it is frozen.
        if (landmark_node.second.frozen) {
          problem->SetParameterBlockConstant(
              C_landmarks->at(landmark_id).translation());
          problem->SetParameterBlockConstant(
              C_landmarks->at(landmark_id).rotation());
        }
      }
      problem->AddResidualBlock(
          LandmarkCostFunction2D::CreateAutoDiffCostFunction(
              observation, prev->data, next->data),
          new ceres::HuberLoss(huber_scale), prev_node_pose->data(),
          next_node_pose->data(), C_landmarks->at(landmark_id).rotation(),
          C_landmarks->at(landmark_id).translation());
    }
  }
}

}  // namespace

OptimizationProblem2D::OptimizationProblem2D(
    const proto::OptimizationProblemOptions& options)
    : options_(options) {}

OptimizationProblem2D::~OptimizationProblem2D() {}

// 2D优化中不使用IMU数据, 所以我们忽略这部分接口
void OptimizationProblem2D::AddImuData(const int trajectory_id,
                                       const sensor::ImuData& imu_data) {
  // IMU data is not used in 2D optimization, so we ignore this part of the
  // interface.
}

// 添加里程计数据
void OptimizationProblem2D::AddOdometryData(
    const int trajectory_id, const sensor::OdometryData& odometry_data) {
  odometry_data_.Append(trajectory_id, odometry_data);
}

// 添加gps数据
void OptimizationProblem2D::AddFixedFramePoseData(
    const int trajectory_id,
    const sensor::FixedFramePoseData& fixed_frame_pose_data) {
  fixed_frame_pose_data_.Append(trajectory_id, fixed_frame_pose_data);
}

// 向优化问题中加入节点位姿数据
void OptimizationProblem2D::AddTrajectoryNode(const int trajectory_id,
                                              const NodeSpec2D& node_data) {
  node_data_.Append(trajectory_id, node_data);
  trajectory_data_[trajectory_id];
}

// 设置trajectory_data_
void OptimizationProblem2D::SetTrajectoryData(
    int trajectory_id, const TrajectoryData& trajectory_data) {
  trajectory_data_[trajectory_id] = trajectory_data;
}

void OptimizationProblem2D::InsertTrajectoryNode(const NodeId& node_id,
                                                 const NodeSpec2D& node_data) {
  node_data_.Insert(node_id, node_data);
  trajectory_data_[node_id.trajectory_id];
}

void OptimizationProblem2D::TrimTrajectoryNode(const NodeId& node_id) {
  empty_imu_data_.Trim(node_data_, node_id);
  odometry_data_.Trim(node_data_, node_id);
  fixed_frame_pose_data_.Trim(node_data_, node_id);
  node_data_.Trim(node_id);
  if (node_data_.SizeOfTrajectoryOrZero(node_id.trajectory_id) == 0) {
    trajectory_data_.erase(node_id.trajectory_id);
  }
}

// 添加子图位姿
void OptimizationProblem2D::AddSubmap(
    const int trajectory_id, const transform::Rigid2d& global_submap_pose) {
  submap_data_.Append(trajectory_id, SubmapSpec2D{global_submap_pose});
}

// 添加子图位姿
void OptimizationProblem2D::InsertSubmap(
    const SubmapId& submap_id, const transform::Rigid2d& global_submap_pose) {
  submap_data_.Insert(submap_id, SubmapSpec2D{global_submap_pose});
}

// 删除指定id的子图位姿
void OptimizationProblem2D::TrimSubmap(const SubmapId& submap_id) {
  submap_data_.Trim(submap_id);
}

// 设置最大迭代次数
void OptimizationProblem2D::SetMaxNumIterations(
    const int32 max_num_iterations) {
  options_.mutable_ceres_solver_options()->set_max_num_iterations(
      max_num_iterations);
}

/**
 * @brief 
 * 
 * @param[in] constraints 所有的约束数据
 * @param[in] trajectories_state 轨迹的状态
 * @param[in] landmark_nodes landmark数据
 */
void OptimizationProblem2D::Solve(
    const std::vector<Constraint>& constraints,
    const std::map<int, PoseGraphInterface::TrajectoryState>&
        trajectories_state,
    const std::map<std::string, LandmarkNode>& landmark_nodes) {
  if (node_data_.empty()) {
    // Nothing to optimize.
    return;
  }

  // 记录下所有FROZEN状态的轨迹
  std::set<int> frozen_trajectories;
  for (const auto& it : trajectories_state) {
    if (it.second == PoseGraphInterface::TrajectoryState::FROZEN) {
      frozen_trajectories.insert(it.first);
    }
  }

  // 创建优化问题对象
  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  // Set the starting point.
  // TODO(hrapp): Move ceres data into SubmapSpec.
  MapById<SubmapId, std::array<double, 3>> C_submaps;
  MapById<NodeId, std::array<double, 3>> C_nodes;
  std::map<std::string, CeresPose> C_landmarks;
  bool first_submap = true;

  // 将需要优化的子图位姿 设置为优化参数 告知对象problem
  // submap_id_data的类型是 IdDataReference
  for (const auto& submap_id_data : submap_data_) {
    // submap_id的轨迹 是否是 已经冻结的轨迹
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
    // 将数据设置成ceres需要的格式
    C_submaps.Insert(submap_id_data.id,
                     FromPose(submap_id_data.data.global_pose));
    // 向优化器中插入需要优化的数据
    // c++11: std::array::data() 返回指向数组对象中第一个元素的指针
    problem.AddParameterBlock(C_submaps.at(submap_id_data.id).data(), 3);

    // 如果是第一幅子图, 或者是已经冻结的轨迹中的子图, Ceres在迭代求解的过程中将不会改变这些参数
    if (first_submap || frozen) {
      first_submap = false;
      // Fix the pose of the first submap or all submaps of a frozen
      // trajectory.
      // 通过SetParameterBlockConstant将对应的参数设定为常量
      problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
    }
  }
  
  // 将需要优化的节点位姿 设置为优化参数 告知对象problem
  for (const auto& node_id_data : node_data_) {
    const bool frozen =
        frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
    C_nodes.Insert(node_id_data.id, FromPose(node_id_data.data.global_pose_2d));
    problem.AddParameterBlock(C_nodes.at(node_id_data.id).data(), 3);
    if (frozen) {
      problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).data());
    }
  }
  
  // Add cost functions for intra- and inter-submap constraints.
  // Step: 第一种残差 将约束作为残差项加入到优化问题中
  for (const Constraint& constraint : constraints) {
    problem.AddResidualBlock(
        // 函数CreateAutoDiffSpaCostFunction用于提供对应约束的SPA代价计算
        CreateAutoDiffSpaCostFunction(constraint.pose),
        // Loop closure constraints should have a loss function.
        // 如果是通过闭环检测构建的约束, 则为之提供一个Huber的核函数
        // 用于降低错误的闭环检测对最终的优化结果带来的负面影响
        constraint.tag == Constraint::INTER_SUBMAP
            ? new ceres::HuberLoss(options_.huber_scale())
            : nullptr,
        C_submaps.at(constraint.submap_id).data(),
        C_nodes.at(constraint.node_id).data());
  }
  
  // Add cost functions for landmarks.
  // Step: 第二种残差 将landmark添加代价函数
  AddLandmarkCostFunctions(landmark_nodes, node_data_, &C_nodes, &C_landmarks,
                           &problem, options_.huber_scale());
  
  // Add penalties for violating odometry or changes between consecutive nodes
  // if odometry is not available.
  // 如果里程计不可用, 则添加违反里程计的处罚或连续节点之间的更改

  // 遍历多个轨迹
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    // 获取每个节点的trajectory——id
    const int trajectory_id = node_it->id.trajectory_id;
    // 获取这trajectoryid的最后一个node
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    // 如果本轨迹是锁定的, 则无需处理跳过
    if (frozen_trajectories.count(trajectory_id) != 0) {
      node_it = trajectory_end;
      continue;
    }

    // 遍历一个轨迹的所有节点, 添加前后两帧节点的相对位姿的残差项
    auto prev_node_it = node_it;
    for (++node_it; node_it != trajectory_end; ++node_it) {
      const NodeId first_node_id = prev_node_it->id;
      const NodeSpec2D& first_node_data = prev_node_it->data;
      prev_node_it = node_it;

      const NodeId second_node_id = node_it->id;
      const NodeSpec2D& second_node_data = node_it->data;

      // 如果节点的索引不连续, 跳过
      if (second_node_id.node_index != first_node_id.node_index + 1) {
        continue;
      }

      // Add a relative pose constraint based on the odometry (if available).
      // 获取相邻两个node的 相对里程计位置差
      std::unique_ptr<transform::Rigid3d> relative_odometry =
          CalculateOdometryBetweenNodes(trajectory_id, first_node_data,
                                        second_node_data);
      // Step: 第三种残差 如果存在里程计则可增加一个里程计残差, 
      if (relative_odometry != nullptr) {
        problem.AddResidualBlock(
            CreateAutoDiffSpaCostFunction(Constraint::Pose{
                *relative_odometry, options_.odometry_translation_weight(),
                options_.odometry_rotation_weight()}),
            nullptr /* loss function */, C_nodes.at(first_node_id).data(),
            C_nodes.at(second_node_id).data());
      }

      // Add a relative pose constraint based on consecutive local SLAM poses.
      // 添加基于连续局部 SLAM 姿势的相对姿势约束
      const transform::Rigid3d relative_local_slam_pose =
          transform::Embed3D(first_node_data.local_pose_2d.inverse() *
                             second_node_data.local_pose_2d);
      // Step: 第四种残差 以 前后两帧节点在local坐标系下的相对位姿 作为残差
      problem.AddResidualBlock(
          CreateAutoDiffSpaCostFunction(
              Constraint::Pose{relative_local_slam_pose,
                               options_.local_slam_pose_translation_weight(),
                               options_.local_slam_pose_rotation_weight()}),
          nullptr /* loss function */, C_nodes.at(first_node_id).data(),
          C_nodes.at(second_node_id).data());
    }
  }

  // 遍历多个轨迹
  std::map<int, std::array<double, 3>> C_fixed_frames;
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    const int trajectory_id = node_it->id.trajectory_id;
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    if (!fixed_frame_pose_data_.HasTrajectory(trajectory_id)) {
      node_it = trajectory_end;
      continue;
    }

    const TrajectoryData& trajectory_data = trajectory_data_.at(trajectory_id);
    bool fixed_frame_pose_initialized = false;
    
    // 遍历一个轨迹的所有节点
    for (; node_it != trajectory_end; ++node_it) {
      const NodeId node_id = node_it->id;
      const NodeSpec2D& node_data = node_it->data;

      // 根据节点的时间对gps数据进行插值
      const std::unique_ptr<transform::Rigid3d> fixed_frame_pose =
          Interpolate(fixed_frame_pose_data_, trajectory_id, node_data.time);
      if (fixed_frame_pose == nullptr) {
        continue;
      }

      const Constraint::Pose constraint_pose{
          *fixed_frame_pose, options_.fixed_frame_pose_translation_weight(),
          options_.fixed_frame_pose_rotation_weight()};

      // 计算初始位姿
      if (!fixed_frame_pose_initialized) {
        transform::Rigid2d fixed_frame_pose_in_map;

        if (trajectory_data.fixed_frame_origin_in_map.has_value()) {
          fixed_frame_pose_in_map = transform::Project2D(
              trajectory_data.fixed_frame_origin_in_map.value());
        } else {
          fixed_frame_pose_in_map =
              node_data.global_pose_2d *
              transform::Project2D(constraint_pose.zbar_ij).inverse();
        }

        // 将这个初始位姿加入到ceres需要的格式数据中
        C_fixed_frames.emplace(trajectory_id,
                               FromPose(fixed_frame_pose_in_map));
        fixed_frame_pose_initialized = true;
      }

      // Step: 第五种残差 
      // ?: 节点与gps第一帧间的坐标变换
      problem.AddResidualBlock(
          CreateAutoDiffSpaCostFunction(constraint_pose),
          options_.fixed_frame_pose_use_tolerant_loss()
              ? new ceres::TolerantLoss(
                    options_.fixed_frame_pose_tolerant_loss_param_a(),
                    options_.fixed_frame_pose_tolerant_loss_param_b())
              : nullptr,
          C_fixed_frames.at(trajectory_id).data(), C_nodes.at(node_id).data());
    }
  }

  // Solve. 进行求解
  ceres::Solver::Summary summary;
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);

  // 如果开启了优化的log输出
  // tag: 展示 这个参数在哪, 什么意思
  if (options_.log_solver_summary()) {
    LOG(INFO) << summary.FullReport();
  }

  // Store the result.
  for (const auto& C_submap_id_data : C_submaps) {
    submap_data_.at(C_submap_id_data.id).global_pose =
        ToPose(C_submap_id_data.data);
  }
  for (const auto& C_node_id_data : C_nodes) {
    node_data_.at(C_node_id_data.id).global_pose_2d =
        ToPose(C_node_id_data.data);
  }
  for (const auto& C_fixed_frame : C_fixed_frames) {
    trajectory_data_.at(C_fixed_frame.first).fixed_frame_origin_in_map =
        transform::Embed3D(ToPose(C_fixed_frame.second));
  }
  for (const auto& C_landmark : C_landmarks) {
    landmark_data_[C_landmark.first] = C_landmark.second.ToRigid();
  }
}

// 
std::unique_ptr<transform::Rigid3d> OptimizationProblem2D::InterpolateOdometry(
    const int trajectory_id, const common::Time time) const {
  // 找到node最近的两帧轮速计(一前一后)
  const auto it = odometry_data_.lower_bound(trajectory_id, time);
  if (it == odometry_data_.EndOfTrajectory(trajectory_id)) {
    return nullptr;
  }

  if (it == odometry_data_.BeginOfTrajectory(trajectory_id)) {
    if (it->time == time) {
      return absl::make_unique<transform::Rigid3d>(it->pose);
    }
    return nullptr;
  }

  const auto prev_it = std::prev(it);
  // 根据时间线性插值
  return absl::make_unique<transform::Rigid3d>(
      Interpolate(transform::TimestampedTransform{prev_it->time, prev_it->pose},
                  transform::TimestampedTransform{it->time, it->pose}, time)
          .transform);
}

/**
 * @brief 两个节点间的相对坐标变换 // 轮速计插值计算位姿
 * 
 * @param[in] trajectory_id 轨迹的id
 * @param[in] first_node_data 前一个节点数据
 * @param[in] second_node_data 后一个节点数据
 * @return std::unique_ptr<transform::Rigid3d> 两个节点的坐标变换
 */
std::unique_ptr<transform::Rigid3d>
OptimizationProblem2D::CalculateOdometryBetweenNodes(
    const int trajectory_id, const NodeSpec2D& first_node_data,
    const NodeSpec2D& second_node_data) const {

  if (odometry_data_.HasTrajectory(trajectory_id)) {
    // 轮速计插值得到第一个node的位姿
    const std::unique_ptr<transform::Rigid3d> first_node_odometry =
        InterpolateOdometry(trajectory_id, first_node_data.time);
    // 轮速计插值得到第二个node的位姿
    const std::unique_ptr<transform::Rigid3d> second_node_odometry =
        InterpolateOdometry(trajectory_id, second_node_data.time);

    if (first_node_odometry != nullptr && second_node_odometry != nullptr) {
      // 两个节点间的相对坐标变化
      // 需要注意的是, 实际上在optimization_problem中, node的位姿都是不带重力对齐的, 
      // 而odometry的pose是带重力对齐的, 因此, 需要将轮速计插值出来的位姿减掉重力对齐
      transform::Rigid3d relative_odometry =
          transform::Rigid3d::Rotation(first_node_data.gravity_alignment) *
          first_node_odometry->inverse() * (*second_node_odometry) *
          transform::Rigid3d::Rotation(
              second_node_data.gravity_alignment.inverse());

      return absl::make_unique<transform::Rigid3d>(relative_odometry);
    }
  }

  return nullptr;
}

}  // namespace optimization
}  // namespace mapping
}  // namespace cartographer

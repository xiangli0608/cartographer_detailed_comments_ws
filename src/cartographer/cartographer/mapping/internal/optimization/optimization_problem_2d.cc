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

// 根据节点的时间对gps数据进行插值, 获取这个时刻的gps数据的位姿 For fixed frame pose.
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
  // 对这个时间一前一后的gps数据进行插值
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
// Rigid2d转array数组
std::array<double, 3> FromPose(const transform::Rigid2d& pose) {
  return {{pose.translation().x(), pose.translation().y(),
           pose.normalized_angle()}};
}

// Converts a pose as represented for Ceres back to an transform::Rigid2d pose.
// array数组转transform::Rigid2d 
transform::Rigid2d ToPose(const std::array<double, 3>& values) {
  return transform::Rigid2d({values[0], values[1]}, values[2]);
}

// Selects a trajectory node closest in time to the landmark observation and
// applies a relative transform from it.
// 根据landmark数据的时间对2个节点位姿进行插值, 得到这个时刻的global坐标系下的位姿
transform::Rigid3d GetInitialLandmarkPose(
    const LandmarkNode::LandmarkObservation& observation,
    const NodeSpec2D& prev_node, const NodeSpec2D& next_node,
    const std::array<double, 3>& prev_node_pose,
    const std::array<double, 3>& next_node_pose) {
  const double interpolation_parameter =
      common::ToSeconds(observation.time - prev_node.time) /
      common::ToSeconds(next_node.time - prev_node.time);
  // 根据landmark数据的时间对2个节点位姿进行插值, 得到这个时刻的tracking_frame在global坐标系下的位姿
  const std::tuple<std::array<double, 4>, std::array<double, 3>>
      rotation_and_translation =
          InterpolateNodes2D(prev_node_pose.data(), prev_node.gravity_alignment,
                             next_node_pose.data(), next_node.gravity_alignment,
                             interpolation_parameter);
  // 将landmark的数据从tracking_frame下的位姿转到global坐标系下
  return transform::Rigid3d::FromArrays(std::get<0>(rotation_and_translation),
                                        std::get<1>(rotation_and_translation)) *
         observation.landmark_to_tracking_transform;
}

// landmark数据 与 通过2个节点位姿插值出来的相对位姿 的差值作为残差项
void AddLandmarkCostFunctions(
    const std::map<std::string, LandmarkNode>& landmark_nodes,
    const MapById<NodeId, NodeSpec2D>& node_data,
    MapById<NodeId, std::array<double, 3>>* C_nodes,
    std::map<std::string, CeresPose>* C_landmarks, ceres::Problem* problem,
    double huber_scale) {
  // 对所有的landmark进行遍历, 添加landmark的残差
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
      // 找到landmark观测时间后的第一个节点
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
      // 找到landmark观测时间前一个节点
      auto prev = std::prev(next);
      // Add parameter blocks for the landmark ID if they were not added before.
      std::array<double, 3>* prev_node_pose = &C_nodes->at(prev->id);
      std::array<double, 3>* next_node_pose = &C_nodes->at(next->id);
      // 如果landmark_id对应的数据没放入到C_landmarks中, 在这添加进去
      if (!C_landmarks->count(landmark_id)) {
        // 如果有优化后的位姿就用优化后的位姿, 没有就根据时间插值算出来一个位姿
        // starting_point就是这帧landmark数据对应的tracking_frame在global坐标系下的位姿
        const transform::Rigid3d starting_point =
            landmark_node.second.global_landmark_pose.has_value()
                ? landmark_node.second.global_landmark_pose.value()
                : GetInitialLandmarkPose(observation, prev->data, next->data,
                                         *prev_node_pose, *next_node_pose);
        // 将landmark数据放入C_landmarks
        C_landmarks->emplace(
            landmark_id,
            // 将landmark数据对应的节点的平移与旋转作为优化变量加入到problem中
            CeresPose(starting_point, 
                      nullptr /* translation_parametrization */,
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
      // Step: 第二种残差 landmark数据与节点位姿间的相对坐标变换 与 landmark观测 的差值作为残差项
      problem->AddResidualBlock(
          LandmarkCostFunction2D::CreateAutoDiffCostFunction(
              observation, prev->data, next->data),
          new ceres::HuberLoss(huber_scale), 
          prev_node_pose->data(),
          next_node_pose->data(), 
          C_landmarks->at(landmark_id).rotation(),
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

// 设置TrajectoryData 只在PoseGraph2D::SetTrajectoryDataFromProto调用了一次
void OptimizationProblem2D::SetTrajectoryData(
    int trajectory_id, const TrajectoryData& trajectory_data) {
  trajectory_data_[trajectory_id] = trajectory_data;
}

// 设置节点位姿
void OptimizationProblem2D::InsertTrajectoryNode(const NodeId& node_id,
                                                 const NodeSpec2D& node_data) {
  node_data_.Insert(node_id, node_data);
  trajectory_data_[node_id.trajectory_id];
}

// 删除节点, 在纯定位时使用
void OptimizationProblem2D::TrimTrajectoryNode(const NodeId& node_id) {
  // 根据这个节点的时间戳删除传感器数据
  empty_imu_data_.Trim(node_data_, node_id);
  odometry_data_.Trim(node_data_, node_id);
  fixed_frame_pose_data_.Trim(node_data_, node_id);
  // 删除节点
  node_data_.Trim(node_id);
  // 如果数据空了就吧轨迹删除掉
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

// 删除指定id的子图位姿, 在纯定位时使用
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
 * @brief 搭建优化问题并进行求解
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

  // 记录下所有FROZEN状态的轨迹id
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
  // ceres需要double的指针, std::array能转成原始指针的形式
  MapById<SubmapId, std::array<double, 3>> C_submaps;
  MapById<NodeId, std::array<double, 3>> C_nodes;
  std::map<std::string, CeresPose> C_landmarks;
  bool first_submap = true;

  // 将需要优化的子图位姿设置为优化参数
  for (const auto& submap_id_data : submap_data_) {
    // submap_id的轨迹 是否是 已经冻结的轨迹
    const bool frozen =
        frozen_trajectories.count(submap_id_data.id.trajectory_id) != 0;
    // 将子图的global_pose放入C_submaps中
    C_submaps.Insert(submap_id_data.id,
                     FromPose(submap_id_data.data.global_pose));
    // c++11: std::array::data() 返回指向数组对象中第一个元素的指针
    // Step: 添加需要优化的数据 这里显式添加参数块,会进行额外的参数块正确性检查
    problem.AddParameterBlock(C_submaps.at(submap_id_data.id).data(), 3);

    if (first_submap || frozen) {
      first_submap = false;
      // Fix the pose of the first submap or all submaps of a frozen
      // trajectory.
      // Step: 如果是第一幅子图, 或者是已经冻结的轨迹中的子图, 不优化这个子图位姿
      problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
    }
  }
  
  // 将需要优化的节点位姿设置为优化参数
  for (const auto& node_id_data : node_data_) {
    const bool frozen =
        frozen_trajectories.count(node_id_data.id.trajectory_id) != 0;
    // 将节点的global_pose_2d放入C_nodes中
    C_nodes.Insert(node_id_data.id, FromPose(node_id_data.data.global_pose_2d));
    problem.AddParameterBlock(C_nodes.at(node_id_data.id).data(), 3);
    // 第一个节点的位姿也是要优化的变量, 不是固定的
    if (frozen) {
      problem.SetParameterBlockConstant(C_nodes.at(node_id_data.id).data());
    }
  }
  
  // Step: 第一种残差 将节点与子图原点在global坐标系下的相对位姿 与 约束 的差值作为残差项
  // Add cost functions for intra- and inter-submap constraints.
  for (const Constraint& constraint : constraints) {
    problem.AddResidualBlock(
        // 根据SPA论文中的公式计算出的残差的CostFunction
        CreateAutoDiffSpaCostFunction(constraint.pose),
        // Loop closure constraints should have a loss function.
        // 为闭环约束提供一个Huber的核函数,用于降低错误的闭环检测对最终的优化结果带来的负面影响
        constraint.tag == Constraint::INTER_SUBMAP // 核函数
            ? new ceres::HuberLoss(options_.huber_scale()) // param: huber_scale
            : nullptr,
        C_submaps.at(constraint.submap_id).data(), // 2个优化变量
        C_nodes.at(constraint.node_id).data());
  }
  
  // Add cost functions for landmarks.
  // Step: landmark数据 与 通过2个节点位姿插值出来的相对位姿 的差值作为残差项
  AddLandmarkCostFunctions(landmark_nodes, node_data_, &C_nodes, &C_landmarks,
                           &problem, options_.huber_scale());
  
  // Add penalties for violating odometry or changes between consecutive nodes
  // if odometry is not available.
  // 如果里程计不可用, 则添加违反里程计的处罚或连续节点之间的更改

  // 遍历多个轨迹, 添加里程计与local结果的残差
  for (auto node_it = node_data_.begin(); node_it != node_data_.end();) {
    // 获取每个节点的轨迹id
    const int trajectory_id = node_it->id.trajectory_id;
    // 获取这条轨迹的最后一个位置的迭代器
    const auto trajectory_end = node_data_.EndOfTrajectory(trajectory_id);
    // 如果轨迹是frozen的, 则无需处理直接跳过
    if (frozen_trajectories.count(trajectory_id) != 0) {
      node_it = trajectory_end;
      continue;
    }

    auto prev_node_it = node_it;
    // 遍历一个轨迹的所有节点, 添加里程计与local结果的残差
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
      // 根据里程计数据进行插值得到的2个节点间的坐标变换
      std::unique_ptr<transform::Rigid3d> relative_odometry =
          CalculateOdometryBetweenNodes(trajectory_id, first_node_data,
                                        second_node_data);
      // Step: 第三种残差 节点与节点间在global坐标系下的相对坐标变换 与 通过里程计数据插值出的相对坐标变换 的差值作为残差项
      // 如果存在里程计则可增加一个残差
      if (relative_odometry != nullptr) {
        problem.AddResidualBlock(
            CreateAutoDiffSpaCostFunction(Constraint::Pose{
                *relative_odometry, options_.odometry_translation_weight(),
                options_.odometry_rotation_weight()}),
            nullptr /* loss function */, 
            C_nodes.at(first_node_id).data(),
            C_nodes.at(second_node_id).data());
      }

      // Add a relative pose constraint based on consecutive local SLAM poses.
      // 计算相邻2个节点在local坐标系下的坐标变换
      const transform::Rigid3d relative_local_slam_pose =
          transform::Embed3D(first_node_data.local_pose_2d.inverse() *
                             second_node_data.local_pose_2d);
      // Step: 第四种残差 节点与节点间在global坐标系下的相对坐标变换 与 相邻2个节点在local坐标系下的相对坐标变换 的差值作为残差项
      problem.AddResidualBlock(
          CreateAutoDiffSpaCostFunction(
              Constraint::Pose{relative_local_slam_pose,
                               options_.local_slam_pose_translation_weight(),
                               options_.local_slam_pose_rotation_weight()}),
          nullptr /* loss function */, 
          C_nodes.at(first_node_id).data(),
          C_nodes.at(second_node_id).data());
    }
  }

  // 遍历多个轨迹, 添加gps的残差
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
    
    // 遍历一个轨迹的所有节点, 添加gps的残差
    for (; node_it != trajectory_end; ++node_it) {
      const NodeId node_id = node_it->id;
      const NodeSpec2D& node_data = node_it->data;

      // 根据节点的时间对gps数据进行插值, 获取这个时刻的gps数据的位姿
      const std::unique_ptr<transform::Rigid3d> fixed_frame_pose =
          Interpolate(fixed_frame_pose_data_, trajectory_id, node_data.time);
      // 要找到第一个有效的数据才能进行残差的计算
      if (fixed_frame_pose == nullptr) {
        continue;
      }
      // gps数据到gps第一个数据间的坐标变换
      const Constraint::Pose constraint_pose{
          *fixed_frame_pose, options_.fixed_frame_pose_translation_weight(),
          options_.fixed_frame_pose_rotation_weight()};

      // 计算gps坐标系原点在global坐标系下的坐标
      if (!fixed_frame_pose_initialized) {
        transform::Rigid2d fixed_frame_pose_in_map;
        // 如果设置了gps数据的原点
        if (trajectory_data.fixed_frame_origin_in_map.has_value()) {
          fixed_frame_pose_in_map = transform::Project2D(
              trajectory_data.fixed_frame_origin_in_map.value());
        } 
        // 第一次优化之前执行的是这里
        else {
          // 计算gps第一个数据在global坐标系下的坐标, 相当于gps数据的坐标系原点在global坐标系下的坐标
          fixed_frame_pose_in_map =
              node_data.global_pose_2d *
              transform::Project2D(constraint_pose.zbar_ij).inverse();
        }

        // 保存gps坐标系原点在global坐标系下的坐标
        C_fixed_frames.emplace(trajectory_id,
                               FromPose(fixed_frame_pose_in_map));
        fixed_frame_pose_initialized = true;
      }

      // Step: 第五种残差 节点与gps坐标系原点在global坐标系下的相对坐标变换 与 通过gps数据进行插值得到的相对坐标变换 的差值作为残差项
      problem.AddResidualBlock(
          CreateAutoDiffSpaCostFunction(constraint_pose),
          options_.fixed_frame_pose_use_tolerant_loss()
              ? new ceres::TolerantLoss(
                    options_.fixed_frame_pose_tolerant_loss_param_a(),
                    options_.fixed_frame_pose_tolerant_loss_param_b())
              : nullptr,
          C_fixed_frames.at(trajectory_id).data(), // 会自动调用AddParameterBlock
          C_nodes.at(node_id).data());
    }
  }

  // Solve. 进行求解
  ceres::Solver::Summary summary;
  ceres::Solve(
      common::CreateCeresSolverOptions(options_.ceres_solver_options()),
      &problem, &summary);

  // 如果开启了优化的log输出, 就输出ceres的报告
  if (options_.log_solver_summary()) {
    LOG(INFO) << summary.FullReport();
  }

  // 将优化后的所有数据进行更新 Store the result.
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

// 根据时间对里程计数据进行插值, 获取这个时刻的里程计数据的位姿
std::unique_ptr<transform::Rigid3d> OptimizationProblem2D::InterpolateOdometry(
    const int trajectory_id, const common::Time time) const {
  // 找到时间上第一个大于等于time的里程计数据的迭代器
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
  // 前一个里程计数据
  const auto prev_it = std::prev(it);
  // 根据时间进行线性插值
  return absl::make_unique<transform::Rigid3d>(
      Interpolate(transform::TimestampedTransform{prev_it->time, prev_it->pose},
                  transform::TimestampedTransform{it->time, it->pose}, time)
          .transform);
}

/**
 * @brief 根据里程计数据算出两个节点间的相对坐标变换
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
    // 插值得到time时刻的里程计数据
    const std::unique_ptr<transform::Rigid3d> first_node_odometry =
        InterpolateOdometry(trajectory_id, first_node_data.time);
    const std::unique_ptr<transform::Rigid3d> second_node_odometry =
        InterpolateOdometry(trajectory_id, second_node_data.time);

    if (first_node_odometry != nullptr && second_node_odometry != nullptr) {
      // 根据里程计数据算出的相对坐标变换
      // 需要注意的是, 实际上在optimization_problem中, node的位姿都是2d平面上的
      // 而odometry的pose是带姿态的, 因此要将轮速计插值出来的位姿转到平面上
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

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

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/eigen_quaterniond_from_two_vectors.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

/**
 * @brief Construct a new Imu Tracker:: Imu Tracker object
 * 
 * @param[in] imu_gravity_time_constant 这个值在2d与3d情况下都为10
 * @param[in] time 
 */
ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()), // 初始方向角
      gravity_vector_(Eigen::Vector3d::UnitZ()),    // 重力方向初始化为[0,0,1]
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}

/**
 * @brief 预测出time时刻的姿态与重力方向
 * 
 * @param[in] time 要预测的时刻
 */
void ImuTracker::Advance(const common::Time time) {
  CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);
  // 上一时刻的角速度乘以时间,得到当前时刻相对于上一时刻的预测的姿态变化量,再转换成四元数
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  // 使用上一时刻的姿态 orientation_ 乘以姿态变化量, 得到当前时刻的预测出的姿态
  orientation_ = (orientation_ * rotation).normalized();

  // 根据预测出的姿态变化量,预测旋转后的线性加速度的值
  gravity_vector_ = rotation.conjugate() * gravity_vector_;
  // 更新时间
  time_ = time;
}

/**
 * @brief 更新线性加速度的值,并根据重力的方向对上一时刻的姿态进行校准
 * 
 * @param[in] imu_linear_acceleration imu的线加速度的大小
 */
void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  // 指数滑动平均法 exponential moving average
 
  // Step: 1 求delta_t, delta_t初始时刻为infinity, 之后为time_-last_linear_acceleration_time_
  const double delta_t =
      last_linear_acceleration_time_ > common::Time::min()
          ? common::ToSeconds(time_ - last_linear_acceleration_time_)
          : std::numeric_limits<double>::infinity();
  last_linear_acceleration_time_ = time_;

  // Step: 2 求alpha, alpha=1-e^(-delta_t/10)
  // delta_t越大, alpha越大
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);

  // Step: 3 将之前的线加速度与当前传入的线加速度进行融合, 这里采用指数滑动平均法

  // 指数来确定权重, 因为有噪声的存在, 时间差越大, 当前的线性加速度的权重越大
  // 这里的gravity_vector_改成线性加速度更清晰一些
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
      
  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  // Step: 4 求得 线性加速度的值 与 由上一时刻姿态求出的线性加速度 间的旋转量
  const Eigen::Quaterniond rotation = FromTwoVectors(
      gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());

  // Step: 5 使用这个旋转量来校准当前的姿态
  orientation_ = (orientation_ * rotation).normalized();

  // note: glog CHECK_GT: 第一个参数要大于第二个参数
  // 如果线性加速度与姿态均计算完全正确,那这二者的乘积应该是 0 0 1
  CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
}

// 更新角速度
void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer

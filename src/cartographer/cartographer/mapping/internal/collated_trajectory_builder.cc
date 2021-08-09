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

#include "cartographer/mapping/internal/collated_trajectory_builder.h"

#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr double kSensorDataRatesLoggingPeriodSeconds = 15.;

}  // namespace

/**
 * @brief Construct a new Collated Trajectory Builder:: Collated Trajectory Builder object
 * 
 * @param[in] trajectory_options 轨迹的参数配置
 * @param[in] sensor_collator 传入的整理传感器的类,有2种类型
 * @param[in] trajectory_id 新生成的轨迹的id
 * @param[in] expected_sensor_ids 所有需要的topic的名字的集合
 * @param[in] wrapped_trajectory_builder 完整的slam GlobalTrajectoryBuilder
 */
CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
    const proto::TrajectoryBuilderOptions& trajectory_options,
    sensor::CollatorInterface* const sensor_collator, const int trajectory_id,
    const std::set<SensorId>& expected_sensor_ids,
    std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder)
    : sensor_collator_(sensor_collator),
      
      // 以下两个参数在 configuration_files/trajectory_builder.lua 中
      // collate_landmarks 为 false, 不要将landmark数据放入到阻塞队列中
      collate_landmarks_(trajectory_options.collate_landmarks()),
      // collate_fixed_frame 为 true, 将gps数据放入阻塞队列中
      collate_fixed_frame_(trajectory_options.collate_fixed_frame()),
      
      trajectory_id_(trajectory_id),
      wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)),
      last_logging_time_(std::chrono::steady_clock::now()) {
        
  // 获取topic的名字, 并根据参数配置决定是否加入LANDMARK与gps的topic
  absl::flat_hash_set<std::string> expected_sensor_id_strings;
  for (const auto& sensor_id : expected_sensor_ids) {
    // collate_landmarks 为 false, sensor_collator_不处理LANDMARK数据
    if (sensor_id.type == SensorId::SensorType::LANDMARK &&
        !collate_landmarks_) {
      continue;
    }
    // collate_fixed_frame 为 true, sensor_collator_处理gps数据
    if (sensor_id.type == SensorId::SensorType::FIXED_FRAME_POSE &&
        !collate_fixed_frame_) {
      continue;
    }
    expected_sensor_id_strings.insert(sensor_id.id);
  }

  // sensor::Collator的初始化
  sensor_collator_->AddTrajectory(
      trajectory_id, expected_sensor_id_strings,
      [this](const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
        HandleCollatedSensorData(sensor_id, std::move(data));
      });
}

// 将数据传入sensor_collator_的AddSensorData进行排序
void CollatedTrajectoryBuilder::AddData(std::unique_ptr<sensor::Data> data) {
  sensor_collator_->AddSensorData(trajectory_id_, std::move(data));
}

/**
 * @brief 处理 按照时间顺序分发出来的传感器数据
 * 
 * @param[in] sensor_id 传感器的topic的名字
 * @param[in] data 需要处理的数据(Data是个类模板,可处理多种不同数据类型的数据)
 */
void CollatedTrajectoryBuilder::HandleCollatedSensorData(
    const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
  auto it = rate_timers_.find(sensor_id);
  // 找不到就新建一个
  if (it == rate_timers_.end()) {
    // map::emplace()返回一个pair
    // emplace().first表示新插入元素或者原始位置的迭代器
    // emplace().second表示插入成功,只有在key在map中不存在时才插入成功
    it = rate_timers_
             .emplace(
                 std::piecewise_construct, 
                 std::forward_as_tuple(sensor_id),
                 std::forward_as_tuple(
                     common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)))
             .first;
  }
  
  // 对数据队列进行更新
  it->second.Pulse(data->GetTime());

  if (std::chrono::steady_clock::now() - last_logging_time_ >
      common::FromSeconds(kSensorDataRatesLoggingPeriodSeconds)) {
    for (const auto& pair : rate_timers_) {
      LOG(INFO) << pair.first << " rate: " << pair.second.DebugString();
    }
    last_logging_time_ = std::chrono::steady_clock::now();
  }

  // 也就是跑carto时候的消息：
  // [ INFO]: collated_trajectory_builder.cc:72] imu rate: 10.00 Hz 1.00e-01 s +/- 4.35e-05 s (pulsed at 100.44% real time)
  // [ INFO]: collated_trajectory_builder.cc:72] scan rate: 19.83 Hz 5.04e-02 s +/- 4.27e-05 s (pulsed at 99.82% real time)

  // 将排序好的数据送入 GlobalTrajectoryBuilder中的AddSensorData()函数中进行使用
  data->AddToTrajectoryBuilder(wrapped_trajectory_builder_.get());
}

}  // namespace mapping
}  // namespace cartographer

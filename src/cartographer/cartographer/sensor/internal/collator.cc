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

#include "cartographer/sensor/internal/collator.h"

namespace cartographer {
namespace sensor {

/**
 * @brief 添加轨迹以生成排序的传感器输出, 每个topic设置一个回调函数
 * 
 * @param[in] trajectory_id 新生成的轨迹的id
 * @param[in] expected_sensor_ids 需要排序的topic名字的集合
 * @param[in] callback 2个参数的回调函数, 实际是CollatedTrajectoryBuilder::HandleCollatedSensorData()函数
 */
void Collator::AddTrajectory(
    const int trajectory_id,
    const absl::flat_hash_set<std::string>& expected_sensor_ids,
    const Callback& callback) {
  for (const auto& sensor_id : expected_sensor_ids) {
    const auto queue_key = QueueKey{trajectory_id, sensor_id};
    queue_.AddQueue(queue_key,
                    // void(std::unique_ptr<Data> data) 带了个默认参数sensor_id
                    [callback, sensor_id](std::unique_ptr<Data> data) {
                      callback(sensor_id, std::move(data));
                    });
    queue_keys_[trajectory_id].push_back(queue_key);
  }
}

// 将 trajectory_id 标记为完成
void Collator::FinishTrajectory(const int trajectory_id) {
  for (const auto& queue_key : queue_keys_[trajectory_id]) {
    queue_.MarkQueueAsFinished(queue_key);
  }
}

// 向数据队列中添加 传感器数据 
void Collator::AddSensorData(const int trajectory_id,
                             std::unique_ptr<Data> data) {
  QueueKey queue_key{trajectory_id, data->GetSensorId()};
  queue_.Add(std::move(queue_key), std::move(data));
}

// 将所有数据队列标记为已完成,分派所有剩下的传感器数据
// 只能调用一次, 在 Flush 之后不能再调用 AddSensorData()
void Collator::Flush() { queue_.Flush(); }

// 返回在 CollatorInterface 解锁之前需要更多数据的轨迹的 ID
// 对于不等待特定轨迹的实现, 返回 'nullopt'
absl::optional<int> Collator::GetBlockingTrajectoryId() const {
  return absl::optional<int>(queue_.GetBlocker().trajectory_id);
}

}  // namespace sensor
}  // namespace cartographer

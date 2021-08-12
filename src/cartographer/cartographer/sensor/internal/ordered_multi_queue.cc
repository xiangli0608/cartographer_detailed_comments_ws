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

#include "cartographer/sensor/internal/ordered_multi_queue.h"

#include <algorithm>
#include <sstream>
#include <vector>

#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace cartographer {
namespace sensor {

namespace {

// Number of items that can be queued up before we log which queues are waiting
// for data.
const int kMaxQueueSize = 500;

}  // namespace

inline std::ostream& operator<<(std::ostream& out, const QueueKey& key) {
  return out << '(' << key.trajectory_id << ", " << key.sensor_id << ')';
}

OrderedMultiQueue::OrderedMultiQueue() {}

OrderedMultiQueue::~OrderedMultiQueue() {
  for (auto& entry : queues_) {
    CHECK(entry.second.finished);
  }
}

/**
 * @brief 添加一个数据队列,并保存回调函数 CollatedTrajectoryBuilder::HandleCollatedSensorData
 * 
 * @param[in] queue_key 轨迹id与topic名字
 * @param[in] callback void(std::unique_ptr<Data> data) 型的函数
 * 这里的callback已经是对应sensor_id的callback了
 */
void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) {
  CHECK_EQ(queues_.count(queue_key), 0);
  queues_[queue_key].callback = std::move(callback);
}

// 将queue_key对应的Queue的finished设置成true
void OrderedMultiQueue::MarkQueueAsFinished(const QueueKey& queue_key) {
  auto it = queues_.find(queue_key);
  CHECK(it != queues_.end()) << "Did not find '" << queue_key << "'.";

  auto& queue = it->second;
  CHECK(!queue.finished);

  queue.finished = true;
  Dispatch();
}

// 向数据队列中添加数据
void OrderedMultiQueue::Add(const QueueKey& queue_key,
                            std::unique_ptr<Data> data) {
  auto it = queues_.find(queue_key);
  // 如果queue_key不在queues_中, 就忽略data
  if (it == queues_.end()) {
    LOG_EVERY_N(WARNING, 1000)
        << "Ignored data for queue: '" << queue_key << "'";
    return;
  }

  // 向数据队列中添加数据
  it->second.queue.Push(std::move(data));

  // 传感器数据的分发处理
  Dispatch();
}

// 将所有处于未完成状态的数据队列标记为完成状态
void OrderedMultiQueue::Flush() {
  // 找到所有unfinished的数据队列
  std::vector<QueueKey> unfinished_queues;
  for (auto& entry : queues_) {
    if (!entry.second.finished) {
      unfinished_queues.push_back(entry.first);
    }
  }
  // 将unfinished_queues标记为完成状态
  for (auto& unfinished_queue : unfinished_queues) {
    MarkQueueAsFinished(unfinished_queue);
  }
}

// 返回阻塞的队列的QueueKey
QueueKey OrderedMultiQueue::GetBlocker() const {
  CHECK(!queues_.empty());
  return blocker_;
}

/**
 * @brief 将处于数据队列中的数据根据时间依次传入回调函数(数据分发)
 * 
 * 3种退出情况:
 * 退出条件1 某个话题的数据队列为空同时又不是完成状态, 就退出
 * 退出条件2 只有多队列queues_为空, 就退出
 * 退出条件3 数据队列中数据的个数只有1个,又不是完成状态,不能确定状态, 就先退出
 */
void OrderedMultiQueue::Dispatch() {
  while (true) {
    /*
      queues_: 
        (0, scan): {      4,     }
        (0, imu):  {1,  3,   5,  }
        (0, odom): {  2,       6,}
    */
    const Data* next_data = nullptr;
    Queue* next_queue = nullptr;
    QueueKey next_queue_key;

    // Step: 1 遍历所有的数据队列, 找到所有数据队列的第一个数据中时间最老的一个数据
    for (auto it = queues_.begin(); it != queues_.end();) {

      // c++11: auto*(指针类型说明符), auto&(引用类型说明符), auto &&(右值引用)

      // 获取当前队列中时间最老的一个的一个数据
      const auto* data = it->second.queue.Peek<Data>();

      if (data == nullptr) {
        // 如果队列已经处于finished状态了, 就删掉这个队列
        if (it->second.finished) {
          queues_.erase(it++);
          continue;
        }
        // 退出条件1: 某个话题的数据队列为空同时又不是完成状态, 就先退出, 发布log并标记为阻塞者
        CannotMakeProgress(it->first);
        return;
      }

      // 第一次进行到这里或者data的时间比next_data的时间小(老数据)
      // 就更新next_data, 并保存当前话题的数据队列以及queue_key
      if (next_data == nullptr || data->GetTime() < next_data->GetTime()) {
        next_data = data;
        next_queue = &it->second;
        next_queue_key = it->first;
      }

      // 数据的时间戳不是按顺序的, 就报错
      CHECK_LE(last_dispatched_time_, next_data->GetTime())
          << "Non-sorted data added to queue: '" << it->first << "'";
      
      ++it;
    } // end for

    // 退出条件2: 只有多队列queues_为空, 才可能next_data==nullptr
    if (next_data == nullptr) {
      CHECK(queues_.empty());
      return;
    }

    // If we haven't dispatched any data for this trajectory yet, fast forward
    // all queues of this trajectory until a common start time has been reached.
    // 如果我们还没有为这个轨迹分配任何数据, 快进这个轨迹的所有队列, 直到达到一个共同的开始时间
    
    // Step: 2 获取对应轨迹id的所有数据队列中的最小共同时间戳, 作为轨迹开始的时间
    const common::Time common_start_time =
        GetCommonStartTime(next_queue_key.trajectory_id);

    // Step: 3 将 next_queue 的时间最老的一个数据传入回调函数进行处理 

    // 大多数情况, 数据时间都会超过common_start_time的
    if (next_data->GetTime() >= common_start_time) {
      // Happy case, we are beyond the 'common_start_time' already.
      // 更新分发数据的时间
      last_dispatched_time_ = next_data->GetTime();
      // 将数据传入 callback() 函数进行处理,并将这个数据从数据队列中删除
      next_queue->callback(next_queue->queue.Pop());
    } 
    // 数据时间小于common_start_time,同时数据队列数据的个数小于2,只有1个数据的情况 罕见
    else if (next_queue->queue.Size() < 2) {
      // 退出条件3: 数据队列数据的个数少,又不是完成状态, 不能确定现在到底是啥情况, 就先退出稍后再处理
      if (!next_queue->finished) {
        // We cannot decide whether to drop or dispatch this yet.
        CannotMakeProgress(next_queue_key);
        return;
      } 
      // 处于完成状态了, 将数据传入 callback() 函数进行最后几个数据的处理
      // 更新分发数据的时间,将数据传入 callback() 进行处理,并将这个数据从数据队列中删除
      last_dispatched_time_ = next_data->GetTime();
      next_queue->callback(next_queue->queue.Pop());
    } 
    // 数据时间小于common_start_time,同时数据队列数据的个数大于等于2个
    else {
      // We take a peek at the time after next data. If it also is not beyond
      // 'common_start_time' we drop 'next_data', otherwise we just found the
      // first packet to dispatch from this queue.

      // 只处理数据在common_start_time的前一个数据, 其他更早的数据会被丢弃掉
      std::unique_ptr<Data> next_data_owner = next_queue->queue.Pop();
      if (next_queue->queue.Peek<Data>()->GetTime() > common_start_time) {
        // 更新分发数据的时间,将数据传入 callback() 进行处理
        last_dispatched_time_ = next_data->GetTime();
        next_queue->callback(std::move(next_data_owner));
      }
    }
  }
}

// 标记queue_key为阻塞者,并按条件发布log,等等这个数据
void OrderedMultiQueue::CannotMakeProgress(const QueueKey& queue_key) {
  // 标记queue_key为阻塞者
  blocker_ = queue_key;
  for (auto& entry : queues_) {
    // queue_key对应的数据队列为空,而某一个传感器数据队列的数据已经大于kMaxQueueSize了
    // 有问题, 进行报错
    if (entry.second.queue.Size() > kMaxQueueSize) {
      // 在该语句第1、61、121……次被执行的时候, 记录日志信息
      LOG_EVERY_N(WARNING, 60) << "Queue waiting for data: " << queue_key;

      // [ WARN] [1628516438.493835120, 1606808659.273453929]: W0809 21:40:38.000000 10662 ordered_multi_queue.cc:230] Queue waiting for data: (0, points2)
      // [ WARN] [1628516439.089736487, 1606808659.869309184]: W0809 21:40:39.000000 10662 ordered_multi_queue.cc:230] Queue waiting for data: (0, points2)
      return;
    }
  }
}

/**
 * @brief 找到数据队列所有第一帧的最大时间(共同时间)
 * 对于某个id的轨迹的 common_start_time 只会计算一次
 * 
 * @param[in] trajectory_id 轨迹id
 * @return common::Time 返回数据队列所有第一帧的最大时间
 */
common::Time OrderedMultiQueue::GetCommonStartTime(const int trajectory_id) {

  // c++11: map::emplace() 返回的 pair 对象
  // pair 的成员变量 first 是一个指向插入元素或阻止插入的元素的迭代器
  // 成员变量 second 是个布尔值, 表示是否插入成功, 如果这个元素的索引已经存在插入会失败,返回false
  auto emplace_result = common_start_time_per_trajectory_.emplace(
      trajectory_id, common::Time::min());
  common::Time& common_start_time = emplace_result.first->second;

  // 如果插入成功了就找到时间戳最大的对common_start_time进行更新, 失败了就不更新
  // 只会在轨迹开始时插入成功一次
  if (emplace_result.second) {
    // 找到这个轨迹下,所有数据队列中数据的时间戳最大 的时间戳
    // 执行到这里时, 所有的数据队列都有值了, 因为没值的情况在Dispatch()中提前返回了
    for (auto& entry : queues_) {
      if (entry.first.trajectory_id == trajectory_id) {
        common_start_time = std::max(
            common_start_time, entry.second.queue.Peek<Data>()->GetTime());
      }
    }
    LOG(INFO) << "All sensor data for trajectory " << trajectory_id
              << " is available starting at '" << common_start_time << "'.";

    // [ INFO] [1628516134.243770381, 1606808649.533687125]: I0809 21:35:34.000000  8604 ordered_multi_queue.cc:264] All sensor data for trajectory 0 is available starting at '637424054495384530'.

  }

  return common_start_time;
}

}  // namespace sensor
}  // namespace cartographer

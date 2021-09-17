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

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_WORK_QUEUE_H
#define CARTOGRAPHER_MAPPING_INTERNAL_WORK_QUEUE_H

#include <chrono>
#include <deque>
#include <functional>

namespace cartographer {
namespace mapping {

struct WorkItem {
  enum class Result {
    kDoNotRunOptimization, // 执行任务结束后不需要执行优化
    kRunOptimization,      // 执行任务结束后需要执行优化
  };

  std::chrono::steady_clock::time_point time;

  // task为 一个函数的名字, 这个函数返回值类型为Result, 参数列表为(), 为空
  std::function<Result()> task;
};

// 双端队列
using WorkQueue = std::deque<WorkItem>;

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_WORK_QUEUE_H

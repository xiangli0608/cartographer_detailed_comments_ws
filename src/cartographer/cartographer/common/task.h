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

#ifndef CARTOGRAPHER_COMMON_TASK_H_
#define CARTOGRAPHER_COMMON_TASK_H_

#include <set>

#include "absl/synchronization/mutex.h"
#include "glog/logging.h"
#include "thread_pool.h"

namespace cartographer {
namespace common {

class ThreadPoolInterface;

class Task {
 public:
  friend class ThreadPoolInterface;

  using WorkItem = std::function<void()>;
  enum State { NEW, DISPATCHED, DEPENDENCIES_COMPLETED, RUNNING, COMPLETED };

  /**
    NEW：新建任务, 还未schedule到线程池
    DISPATCHED： 任务已经schedule 到线程池
    DEPENDENCIES_COMPLETED： 任务依赖已经执行完成
    RUNNING： 任务执行中
    COMPLETED： 任务完成

    对任一个任务的状态转换顺序为：
    NEW->DISPATCHED->DEPENDENCIES_COMPLETED->RUNNING->COMPLETED
  */

  Task() = default;
  ~Task();

  State GetState() LOCKS_EXCLUDED(mutex_);

  // State must be 'NEW'.
  void SetWorkItem(const WorkItem& work_item) LOCKS_EXCLUDED(mutex_);

  // State must be 'NEW'. 'dependency' may be nullptr, in which case it is
  // assumed completed.
  void AddDependency(std::weak_ptr<Task> dependency) LOCKS_EXCLUDED(mutex_);

 private:
  // Allowed in all states.
  void AddDependentTask(Task* dependent_task);

  // State must be 'DEPENDENCIES_COMPLETED' and becomes 'COMPLETED'.
  void Execute() LOCKS_EXCLUDED(mutex_);

  // State must be 'NEW' and becomes 'DISPATCHED' or 'DEPENDENCIES_COMPLETED'.
  void SetThreadPool(ThreadPoolInterface* thread_pool) LOCKS_EXCLUDED(mutex_);

  // State must be 'NEW' or 'DISPATCHED'. If 'DISPATCHED', may become
  // 'DEPENDENCIES_COMPLETED'.
  void OnDependenyCompleted();

  // 需要执行的任务
  WorkItem work_item_ GUARDED_BY(mutex_);
  ThreadPoolInterface* thread_pool_to_notify_ GUARDED_BY(mutex_) = nullptr;
  // 初始状态为NEW
  State state_ GUARDED_BY(mutex_) = NEW;
  // 本任务依赖的任务的个数
  unsigned int uncompleted_dependencies_ GUARDED_BY(mutex_) = 0;
  // 依赖本任务的其他任务
  std::set<Task*> dependent_tasks_ GUARDED_BY(mutex_);

  absl::Mutex mutex_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TASK_H_

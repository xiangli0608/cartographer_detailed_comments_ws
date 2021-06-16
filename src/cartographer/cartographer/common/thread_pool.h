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

#ifndef CARTOGRAPHER_COMMON_THREAD_POOL_H_
#define CARTOGRAPHER_COMMON_THREAD_POOL_H_

#include <deque>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "cartographer/common/task.h"

namespace cartographer {
namespace common {

class Task;

class ThreadPoolInterface {
 public:
  ThreadPoolInterface() {}
  virtual ~ThreadPoolInterface() {}
  virtual std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) = 0;

 protected:
  void Execute(Task* task);
  void SetThreadPool(Task* task);

 private:
  // 声明 Task 为友元类,Task类的所有成员函数就都可以访问ThreadPoolInterface类的对象的私有成员.
  friend class Task;

  virtual void NotifyDependenciesCompleted(Task* task) = 0;
};

// A fixed number of threads working on tasks. Adding a task does not block.
// Tasks may be added whether or not their dependencies are completed.
// When all dependencies of a task are completed, it is queued up for execution
// in a background thread. The queue must be empty before calling the
// destructor. The thread pool will then wait for the currently executing work
// items to finish and then destroy the threads.
// 固定数量的线程处理任务. 添加任务不会阻塞.
// 无论是否完成依赖, 都可以添加任务.
// 当一个任务的所有依赖都完成后, 它会在后台线程中排队等待执行. 
// 在调用析构函数之前队列必须为空. 然后线程池将等待当前正在执行的工作项完成, 然后销毁线程


class ThreadPool : public ThreadPoolInterface {
 public:
  explicit ThreadPool(int num_threads);
  ~ThreadPool();

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  // When the returned weak pointer is expired, 'task' has certainly completed,
  // so dependants no longer need to add it as a dependency.
  // 当返回的弱指针过期时, 'task'肯定已经完成, 因此依赖者不再需要将其添加为依赖项.
  std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task)
      LOCKS_EXCLUDED(mutex_) override;

 private:
  void DoWork();

  void NotifyDependenciesCompleted(Task* task) LOCKS_EXCLUDED(mutex_) override;

  absl::Mutex mutex_;

  // 结束线程的标志
  bool running_ GUARDED_BY(mutex_) = true;
  // 线程池
  std::vector<std::thread> pool_ GUARDED_BY(mutex_);  
  // 准备执行的task
  std::deque<std::shared_ptr<Task>> task_queue_ GUARDED_BY(mutex_);
  // 未准备好的 task, task可能有依赖还未完成
  absl::flat_hash_map<Task*, std::shared_ptr<Task>> tasks_not_ready_
      GUARDED_BY(mutex_);
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_THREAD_POOL_H_

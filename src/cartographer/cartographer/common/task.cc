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

#include "cartographer/common/task.h"

namespace cartographer {
namespace common {

Task::~Task() {
  // TODO(gaschler): Relax some checks after testing.
  if (state_ != NEW && state_ != COMPLETED) {
    LOG(WARNING) << "Delete Task between dispatch and completion.";
  }
}

// 返回本Task当前状态 
Task::State Task::GetState() {
  absl::MutexLock locker(&mutex_);
  return state_;
}

// 设置本Task需要执行的任务 （函数） 
// 状态: NEW
void Task::SetWorkItem(const WorkItem& work_item) {
  absl::MutexLock locker(&mutex_);
  CHECK_EQ(state_, NEW);
  work_item_ = work_item;
}

// c++11: std::weak_ptr weak_ptr被设计为与shared_ptr共同工作, 
// 可以从一个shared_ptr或者另一个weak_ptr对象构造, 获得资源的观测权
// 但weak_ptr没有共享资源, 它的构造不会引起指针引用计数的增加.
// 同样, 在weak_ptr析构时也不会导致引用计数的减少, 它只是一个静静地观察者.
// weak_ptr没有重载operator*和->, 这是特意的, 因为它不共享指针, 不能操作资源, 这是它弱的原因
// 但它可以使用一个非常重要的成员函数lock()从被观测的shared_ptr获得一个可用的shared_ptr对象, 从而操作资源.

// 为本任务添加依赖
void Task::AddDependency(std::weak_ptr<Task> dependency) {
  std::shared_ptr<Task> shared_dependency;
  {
    absl::MutexLock locker(&mutex_);
    CHECK_EQ(state_, NEW);
    // 如果指针指针成功获取对象
    if ((shared_dependency = dependency.lock())) {
      ++uncompleted_dependencies_;
    }
  }
  
  if (shared_dependency) {
    // 将本task加入到shared_dependency的集合dependent_tasks_中
    shared_dependency->AddDependentTask(this);
  }
}

// 将线程池与本任务连接起来, 如果没有未完成的依赖, 则告诉线程池可以将本任务放入到执行队列中
// 状态: NEW -> DISPATCHED || NEW -> DISPATCHED -> DEPENDENCIES_COMPLETED
void Task::SetThreadPool(ThreadPoolInterface* thread_pool) {
  absl::MutexLock locker(&mutex_);
  CHECK_EQ(state_, NEW);

  // 将任务状态设置为 DISPATCHED
  state_ = DISPATCHED;

  // 将thread_pool_to_notify_指针指向传入的thread_pool
  thread_pool_to_notify_ = thread_pool;

  // 如果本Task没有未完成的依赖, 则通知线程池可以将本任务放入到执行队列中
  if (uncompleted_dependencies_ == 0) {
    state_ = DEPENDENCIES_COMPLETED;
    CHECK(thread_pool_to_notify_);
    thread_pool_to_notify_->NotifyDependenciesCompleted(this);
  }
}

// 添加依赖本Task的Task
void Task::AddDependentTask(Task* dependent_task) {
  absl::MutexLock locker(&mutex_);

  // 如果本Task完成了, 那就通知依赖dependent_task
  if (state_ == COMPLETED) {
    dependent_task->OnDependenyCompleted();
    return;
  }
  // 将依赖本任务的任务放入set中
  bool inserted = dependent_tasks_.insert(dependent_task).second;
  CHECK(inserted) << "Given dependency is already a dependency.";
}

// 本任务依赖的任务完成了, 可以将本任务加入到线程池的待处理列表中了
// 状态: DISPATCHED -> DEPENDENCIES_COMPLETED
void Task::OnDependenyCompleted() {
  absl::MutexLock locker(&mutex_);
  CHECK(state_ == NEW || state_ == DISPATCHED);
  // 依赖的任务减一
  --uncompleted_dependencies_;
  if (uncompleted_dependencies_ == 0 && state_ == DISPATCHED) {
    state_ = DEPENDENCIES_COMPLETED;
    CHECK(thread_pool_to_notify_);
    thread_pool_to_notify_->NotifyDependenciesCompleted(this);
  }
}

// 执行本任务, 也就是传入的函数work_item_
// 状态: DEPENDENCIES_COMPLETED -> RUNNING -> COMPLETED
void Task::Execute() {
  {
    absl::MutexLock locker(&mutex_);
    CHECK_EQ(state_, DEPENDENCIES_COMPLETED);
    state_ = RUNNING;
  }

  // Execute the work item.
  if (work_item_) {
    work_item_();
  }

  absl::MutexLock locker(&mutex_);
  state_ = COMPLETED;

  // 通知依赖本任务的其他任务, 本任务执行完了
  for (Task* dependent_task : dependent_tasks_) {
    dependent_task->OnDependenyCompleted();
  }
}

}  // namespace common
}  // namespace cartographer

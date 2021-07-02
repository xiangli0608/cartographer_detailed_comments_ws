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

#include "cartographer_ros/ros_log_sink.h"

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "glog/log_severity.h"
#include "ros/console.h"

namespace cartographer_ros {

namespace {

/**
 * @brief 根据给定的文件全路径名, 获取文件名
 * 
 * @param[in] filepath 
 * @return const char* 返回文件名
 */
const char* GetBasename(const char* filepath) {
  // 找到 '/' 最后一次在filepath中出现的位置
  const char* base = std::strrchr(filepath, '/');
  // 找到'/',就将'/'之后的字符串返回；找不到'/', 就将整个filepath返回
  return base ? (base + 1) : filepath;
}

}  // namespace


/**
 * @brief 在构造函数中调用AddLogSink(), 将ScopedRosLogSink类注册到glog中
 */
ScopedRosLogSink::ScopedRosLogSink() : will_die_(false) { AddLogSink(this); }
ScopedRosLogSink::~ScopedRosLogSink() { RemoveLogSink(this); }

/**
 * @brief 重载了send()方法, 使用ROS_INFO进行glog消息的输出
 * 
 * @param[in] severity 消息级别
 * @param[in] filename 全路径文件名
 * @param[in] base_filename 文件名
 * @param[in] line 消息所在的文件行数
 * @param[in] tm_time 消息的时间
 * @param[in] message 消息数据本体
 * @param[in] message_len 消息长度
 */
void ScopedRosLogSink::send(const ::google::LogSeverity severity,
                            const char* const filename,
                            const char* const base_filename, 
                            const int line,
                            const struct std::tm* const tm_time,
                            const char* const message,
                            const size_t message_len) {
  const std::string message_string = ::google::LogSink::ToString(
      severity, GetBasename(filename), line, tm_time, message, message_len);
  switch (severity) {
    case ::google::GLOG_INFO:
      ROS_INFO_STREAM(message_string);
      break;

    case ::google::GLOG_WARNING:
      ROS_WARN_STREAM(message_string);
      break;

    case ::google::GLOG_ERROR:
      ROS_ERROR_STREAM(message_string);
      break;

    case ::google::GLOG_FATAL:
      ROS_FATAL_STREAM(message_string);
      will_die_ = true;
      break;
  }
}

// WaitTillSent()会在每次send后调用, 用于一些异步写的场景
void ScopedRosLogSink::WaitTillSent() {
  if (will_die_) {
    // Give ROS some time to actually publish our message.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

}  // namespace cartographer_ros

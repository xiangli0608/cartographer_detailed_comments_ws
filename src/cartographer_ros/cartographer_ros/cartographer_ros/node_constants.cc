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

#include "cartographer_ros/node_constants.h"

#include "glog/logging.h"

namespace cartographer_ros {

/**
 * @brief 如果只有一个传感器, 那订阅的topic就是topic, 
 *        如果是多个传感器, 那订阅的topic就是topic_1,topic_2
 * 
 * @param[in] topic 订阅话题的名字
 * @param[in] num_topics 传感器的个数
 * @return std::vector<std::string> 订阅话题的名字的集合
 */
std::vector<std::string> ComputeRepeatedTopicNames(const std::string& topic,
                                                   const int num_topics) {
  CHECK_GE(num_topics, 0);
  if (num_topics == 1) {
    return {topic};
  }
  std::vector<std::string> topics;
  topics.reserve(num_topics);
  for (int i = 0; i < num_topics; ++i) {
    topics.emplace_back(topic + "_" + std::to_string(i + 1));
  }
  // num_topics要是0就返回空的vector
  return topics;
}

}  // namespace cartographer_ros

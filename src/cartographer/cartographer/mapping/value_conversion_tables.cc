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

#include "cartographer/mapping/value_conversion_tables.h"

#include "absl/memory/memory.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace {

constexpr uint16 kUpdateMarker = 1u << 15;

/**
 * @brief 将[0, 1~32767] 映射到 [0.9, 0.1~0.9]
 * 
 * @param[in] value [0, 32767]的值, 0 对应0.9
 * @param[in] unknown_value 0
 * @param[in] unknown_result 0.9 
 * @param[in] lower_bound 0.1 下界
 * @param[in] upper_bound 0.9 上界
 * @return float 转换后的数值
 */
float SlowValueToBoundedFloat(const uint16 value, 
                              const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {
  CHECK_LE(value, 32767);
  if (value == unknown_value) return unknown_result;
  const float kScale = (upper_bound - lower_bound) / 32766.f;
  return value * kScale + (lower_bound - kScale);
}

/**
 * @brief 新建转换表
 * 
 * @param[in] unknown_value 0 
 * @param[in] unknown_result 0.9 
 * @param[in] lower_bound 0.1 
 * @param[in] upper_bound 0.9 
 * @return std::unique_ptr<std::vector<float>> 转换表的指针
 */
std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  auto result = absl::make_unique<std::vector<float>>();
  // num_values = 65536
  size_t num_values = std::numeric_limits<uint16>::max() + 1; 
  // 申请空间
  result->reserve(num_values);

  // 将[0, 1~32767]映射成[0.9, 0.1~0.9]
  // vector的个数为65536, 所以存的是2遍[0-32767]的映射
  for (size_t value = 0; value != num_values; ++value) {
    result->push_back(SlowValueToBoundedFloat(
        static_cast<uint16>(value) & ~kUpdateMarker, // 取右边15位的数据, 0-32767
        unknown_value,
        unknown_result, lower_bound, upper_bound));
  }
  return result;
}
}  // namespace

/**
 * @brief 获取转换表, 这个函数只会调用1次
 * 
 * @param[in] unknown_result 0.9 未知时的值
 * @param[in] lower_bound 0.1 最小correspondence_cost
 * @param[in] upper_bound 0.9 最大correspondence_cost
 * @return const std::vector<float>* 
 */
const std::vector<float>* ValueConversionTables::GetConversionTable(
    float unknown_result, float lower_bound, float upper_bound) {
  // 将bounds作为key
  std::tuple<float, float, float> bounds =
      std::make_tuple(unknown_result, lower_bound, upper_bound);
  auto lookup_table_iterator = bounds_to_lookup_table_.find(bounds);

  // 如果没有bounds这个key就新建
  if (lookup_table_iterator == bounds_to_lookup_table_.end()) {
    // 新建转换表
    auto insertion_result = bounds_to_lookup_table_.emplace(
        bounds, PrecomputeValueToBoundedFloat(0, unknown_result, lower_bound,
                                              upper_bound));
    return insertion_result.first->second.get();
  } 
  // 如果存在就直接返回原始指针
  else {
    return lookup_table_iterator->second.get();
  }
}

}  // namespace mapping
}  // namespace cartographer

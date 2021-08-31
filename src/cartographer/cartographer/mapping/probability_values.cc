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

#include "cartographer/mapping/probability_values.h"

#include "absl/memory/memory.h"

namespace cartographer {
namespace mapping {

namespace {

constexpr int kValueCount = 32768;

// 0 is unknown, [1, 32767] maps to [lower_bound, upper_bound].
// 将[0, 1~32767] 映射成 [0.9, 0.1~0.9]
float SlowValueToBoundedFloat(const uint16 value, const uint16 unknown_value,
                              const float unknown_result,
                              const float lower_bound,
                              const float upper_bound) {
  CHECK_LT(value, kValueCount);
  if (value == unknown_value) return unknown_result;
  const float kScale = (upper_bound - lower_bound) / (kValueCount - 2.f);
  return value * kScale + (lower_bound - kScale);
}

// 新建转换表
std::unique_ptr<std::vector<float>> PrecomputeValueToBoundedFloat(
    const uint16 unknown_value, const float unknown_result,
    const float lower_bound, const float upper_bound) {
  auto result = absl::make_unique<std::vector<float>>();
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  // 重复2遍
  constexpr int kRepetitionCount = 2;
  result->reserve(kRepetitionCount * kValueCount);
  for (int repeat = 0; repeat != kRepetitionCount; ++repeat) {
    for (int value = 0; value != kValueCount; ++value) {
      result->push_back(SlowValueToBoundedFloat(
          value, unknown_value, unknown_result, lower_bound, upper_bound));
    }
  }
  return result;
}

// 返回ValueToProbability转换表的指针
std::unique_ptr<std::vector<float>> PrecomputeValueToProbability() {
  return PrecomputeValueToBoundedFloat(kUnknownProbabilityValue,          // 0
                                       kMinProbability, kMinProbability,  // 0.1, 0.1
                                       kMaxProbability);                  // 0.9
}

// 返回ValueToCorrespondenceCost转换表的指针
std::unique_ptr<std::vector<float>> PrecomputeValueToCorrespondenceCost() {
  return PrecomputeValueToBoundedFloat(
      kUnknownCorrespondenceValue, kMaxCorrespondenceCost,  // 0,   0.9
      kMinCorrespondenceCost, kMaxCorrespondenceCost);      // 0.1, 0.9
}

}  // namespace

// ValueToProbability转换表
const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability().release();

// [0, 1~32767] 映射成 [0.9, 0.1~0.9]转换表
const std::vector<float>* const kValueToCorrespondenceCost =
    PrecomputeValueToCorrespondenceCost().release();

// 将栅格是未知状态与odds状态下, 将更新时的所有可能结果预先计算出来
std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount);
  // 当前cell是unknown情况下直接把 odd转成概率值付给cell
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) +
                   kUpdateMarker); // 加上kUpdateMarker作为一个标志, 代表这个栅格已经被更新了
  // 计算更新时 从1到32768的所有可能的 更新后的结果 
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) +
                     kUpdateMarker);
  }
  return result;
}

// 将栅格是未知状态与odds状态下, 将更新时的所有可能结果预先计算出来
std::vector<uint16> ComputeLookupTableToApplyCorrespondenceCostOdds(
    float odds) {
  std::vector<uint16> result;
  result.reserve(kValueCount); // 32768

  // 当前cell是unknown情况下直接把odds转成value存进来
  result.push_back(CorrespondenceCostToValue(ProbabilityToCorrespondenceCost(
                       ProbabilityFromOdds(odds))) +
                   kUpdateMarker); // 加上kUpdateMarker作为一个标志, 代表这个栅格已经被更新了
  // 计算更新时 从1到32768的所有可能的 更新后的结果 
  for (int cell = 1; cell != kValueCount; ++cell) {
    result.push_back(
        CorrespondenceCostToValue(
            ProbabilityToCorrespondenceCost(ProbabilityFromOdds(
                odds * Odds(CorrespondenceCostToProbability(
                           (*kValueToCorrespondenceCost)[cell]))))) +
        kUpdateMarker);
  }
  return result;
}

}  // namespace mapping
}  // namespace cartographer

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

#ifndef CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_
#define CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_

#include "cartographer/mapping/trajectory_builder_interface.h"
#include "cartographer/sensor/data.h"

namespace cartographer {
namespace sensor {

template <typename DataType>
class Dispatchable : public Data {
 public:
  Dispatchable(const std::string &sensor_id, const DataType &data)
      : Data(sensor_id), data_(data) {}

  common::Time GetTime() const override { return data_.time; }

  // 调用传入的trajectory_builder的AddSensorData()
  void AddToTrajectoryBuilder(
      mapping::TrajectoryBuilderInterface *const trajectory_builder) override {
    trajectory_builder->AddSensorData(sensor_id_, data_);
  }

  const DataType &data() const { return data_; }

 private:
  const DataType data_;
};


// c++11: template <typename DataType> 
// 函数模板的调用使用 实参推演 来进行
// 类模板 模板形参的类型必须在类名后的尖括号中明确指定, 不能使用实参推演 
// 在类外声明一个 函数模板, 使用 实参推演 的方式来使得 类模板可以自动适应不同的数据类型


// 根据传入的data的数据类型,自动推断DataType, 实现一个函数处理不同类型的传感器数据
template <typename DataType>
std::unique_ptr<Dispatchable<DataType>> MakeDispatchable(
    const std::string &sensor_id, const DataType &data) {
  return absl::make_unique<Dispatchable<DataType>>(sensor_id, data);
}

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_INTERNAL_DISPATCHABLE_H_

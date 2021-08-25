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

#ifndef CARTOGRAPHER_MAPPING_ID_H_
#define CARTOGRAPHER_MAPPING_ID_H_

#include <algorithm>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <ostream>
#include <tuple>
#include <vector>

#include "absl/memory/memory.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace internal {

// c++11: decltype 可以从一个变量或表达式中得到类型
// 在 C++11 中, auto可以和decltype一起用, 将auto放置在返回值类型上当做占位符, 不表达实际意思
// 在参数列表后添加 -> decltype( ), 这是后置返回类型, 代表返回的类型是 () 中的类型
// 在 C++14 中, 则可以不用 decltype

template <class T>
auto GetTimeImpl(const T& t, int) -> decltype(t.time()) {
  return t.time();
}
template <class T>
auto GetTimeImpl(const T& t, unsigned) -> decltype(t.time) {
  return t.time;
}
template <class T>
common::Time GetTime(const T& t) {
  return GetTimeImpl(t, 0);
}

}  // namespace internal

// Uniquely identifies a trajectory node using a combination of a unique
// trajectory ID and a zero-based index of the node inside that trajectory.
// 使用唯一的轨迹ID和该轨迹内节点的从零开始的索引的组合来唯一地标识轨迹节点.
// 一个是轨迹ID, 一个是节点的序号
struct NodeId {
  NodeId(int trajectory_id, int node_index)
      : trajectory_id(trajectory_id), node_index(node_index) {}

  int trajectory_id;
  int node_index;

  bool operator==(const NodeId& other) const {
    return std::forward_as_tuple(trajectory_id, node_index) ==
           std::forward_as_tuple(other.trajectory_id, other.node_index);
  }

  bool operator!=(const NodeId& other) const { return !operator==(other); }

  bool operator<(const NodeId& other) const {
    return std::forward_as_tuple(trajectory_id, node_index) <
           std::forward_as_tuple(other.trajectory_id, other.node_index);
  }

  void ToProto(proto::NodeId* proto) const {
    proto->set_trajectory_id(trajectory_id);
    proto->set_node_index(node_index);
  }
};

inline std::ostream& operator<<(std::ostream& os, const NodeId& v) {
  return os << "(" << v.trajectory_id << ", " << v.node_index << ")";
}

// Uniquely identifies a submap using a combination of a unique trajectory ID
// and a zero-based index of the submap inside that trajectory.
// 使用唯一的轨迹ID和该轨迹内子图的从零开始的索引的组合来唯一地标识子图.
// 一个是轨迹ID, 一个是子图的序号
struct SubmapId {
  SubmapId(int trajectory_id, int submap_index)
      : trajectory_id(trajectory_id), submap_index(submap_index) {}

  int trajectory_id;
  int submap_index;

  bool operator==(const SubmapId& other) const {
    return std::forward_as_tuple(trajectory_id, submap_index) ==
           std::forward_as_tuple(other.trajectory_id, other.submap_index);
  }

  bool operator!=(const SubmapId& other) const { return !operator==(other); }

  bool operator<(const SubmapId& other) const {
    return std::forward_as_tuple(trajectory_id, submap_index) <
           std::forward_as_tuple(other.trajectory_id, other.submap_index);
  }

  void ToProto(proto::SubmapId* proto) const {
    proto->set_trajectory_id(trajectory_id);
    proto->set_submap_index(submap_index);
  }
};

inline std::ostream& operator<<(std::ostream& os, const SubmapId& v) {
  return os << "(" << v.trajectory_id << ", " << v.submap_index << ")";
}

// 保存所有轨迹中的第一个id号与最后一个id号
template <typename IteratorType>
class Range {
 public:
  Range(const IteratorType& begin, const IteratorType& end)
      : begin_(begin), end_(end) {}

  IteratorType begin() const { return begin_; }
  IteratorType end() const { return end_; }

 private:
  IteratorType begin_;
  IteratorType end_;
};

// Reminiscent of std::map, but indexed by 'IdType' which can be 'NodeId' or
// 'SubmapId'.
// Note: This container will only ever contain non-empty trajectories. Trimming
// the last remaining node of a trajectory drops the trajectory.
// std::map的封装 IdType 只能是 NodeId 或者SubmapId, 数据类型都是轨迹id加索引
template <typename IdType, typename DataType>
class MapById {
 private:
  struct MapByIndex;

 public:
  struct IdDataReference {
    IdType id;
    const DataType& data;
  };

  // 自定义迭代器
  class ConstIterator {
   public:
    // c++11: std::bidirectional_iterator_tag 用于将迭代器的类别标识为双向迭代器
    // 一个满足 STL 要求的迭代器类必须全部定义这些别名
    using iterator_category = std::bidirectional_iterator_tag;  // 迭代器类别的标签类类型
    using value_type = IdDataReference;       // 迭代器所指向值的类型
    using difference_type = int64;            // 两个迭代器之间差别值的类型
    using pointer = std::unique_ptr<const IdDataReference>;     // 迭代器所表示的指针类型
    using reference = const IdDataReference&; // 来自于 *ConstIterator 的引用类型

    // 构造函数 通过MapById和轨迹id生成指向轨迹第一个数据的迭代器
    explicit ConstIterator(const MapById& map_by_id, const int trajectory_id)
        : current_trajectory_(
              map_by_id.trajectories_.lower_bound(trajectory_id)),
          end_trajectory_(map_by_id.trajectories_.end()) {
      if (current_trajectory_ != end_trajectory_) {
        current_data_ = current_trajectory_->second.data_.begin();
        // 如果数据不可用就将current_data_设置为
        AdvanceToValidDataIterator();
      }
    }

    // 构造函数 通过MapById和IdType生成指向指定序号节点的迭代器
    explicit ConstIterator(const MapById& map_by_id, const IdType& id)
        : current_trajectory_(map_by_id.trajectories_.find(id.trajectory_id)),
          end_trajectory_(map_by_id.trajectories_.end()) {
      if (current_trajectory_ != end_trajectory_) {
        current_data_ =
            current_trajectory_->second.data_.find(MapById::GetIndex(id));
        // 如果没有这个数据, 就将轨迹设置从end_trajectory_
        if (current_data_ == current_trajectory_->second.data_.end()) {
          current_trajectory_ = end_trajectory_;
        }
      }
    }

    // 重载解引用符号*, 获取IdDataReference格式数据的拷贝
    IdDataReference operator*() const {
      CHECK(current_trajectory_ != end_trajectory_);
      return IdDataReference{
          IdType{current_trajectory_->first, current_data_->first},
          current_data_->second};
    }

    std::unique_ptr<const IdDataReference> operator->() const {
      return absl::make_unique<const IdDataReference>(this->operator*());
    }

    ConstIterator& operator++() {
      CHECK(current_trajectory_ != end_trajectory_);
      ++current_data_;
      AdvanceToValidDataIterator();
      return *this;
    }

    ConstIterator& operator--() {
      while (current_trajectory_ == end_trajectory_ ||
             current_data_ == current_trajectory_->second.data_.begin()) {
        --current_trajectory_;
        current_data_ = current_trajectory_->second.data_.end();
      }
      --current_data_;
      return *this;
    }

    bool operator==(const ConstIterator& it) const {
      if (current_trajectory_ == end_trajectory_ ||
          it.current_trajectory_ == it.end_trajectory_) {
        return current_trajectory_ == it.current_trajectory_;
      }
      return current_trajectory_ == it.current_trajectory_ &&
             current_data_ == it.current_data_;
    }

    bool operator!=(const ConstIterator& it) const { return !operator==(it); }

   private:
    // 如果这个轨迹的数据不存在, 那么移动到下一个trajectory_的begin()
    void AdvanceToValidDataIterator() {
      CHECK(current_trajectory_ != end_trajectory_);
      while (current_data_ == current_trajectory_->second.data_.end()) {
        ++current_trajectory_;
        if (current_trajectory_ == end_trajectory_) {
          return;
        }
        current_data_ = current_trajectory_->second.data_.begin();
      }
    }

    // 当前轨迹 map_by_id.trajectories_lower_bound(trajectory_id)
    typename std::map<int, MapByIndex>::const_iterator current_trajectory_;
    // 轨迹结束的标志 map_by_id.trajectories_.end()
    typename std::map<int, MapByIndex>::const_iterator end_trajectory_;
    // map_by_id.trajectories_[i].MapByIndex.data_.begin() int是数据的索引, DataType是数据
    typename std::map<int, DataType>::const_iterator current_data_;
  };

  // 指向轨迹id的迭代器
  class ConstTrajectoryIterator {
   public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = int;
    using difference_type = int64;
    using pointer = const int*;
    using reference = const int&;

    explicit ConstTrajectoryIterator(
        typename std::map<int, MapByIndex>::const_iterator current_trajectory)
        : current_trajectory_(current_trajectory) {}

    // 获取轨迹的id
    int operator*() const { return current_trajectory_->first; }

    ConstTrajectoryIterator& operator++() {
      ++current_trajectory_;
      return *this;
    }

    ConstTrajectoryIterator& operator--() {
      --current_trajectory_;
      return *this;
    }

    bool operator==(const ConstTrajectoryIterator& it) const {
      return current_trajectory_ == it.current_trajectory_;
    }

    bool operator!=(const ConstTrajectoryIterator& it) const {
      return !operator==(it);
    }

   private:
    typename std::map<int, MapByIndex>::const_iterator current_trajectory_;
  };

  // Appends data to a 'trajectory_id', creating trajectories as needed.
  // 向轨迹的最后添加数据, 返回新添加数据的IdType索引
  IdType Append(const int trajectory_id, const DataType& data) {
    CHECK_GE(trajectory_id, 0);
    auto& trajectory = trajectories_[trajectory_id];
    CHECK(trajectory.can_append_);
    // 找到最后一个元素的index再加1
    const int index =
        trajectory.data_.empty() ? 0 : trajectory.data_.rbegin()->first + 1;
    // 加入到trajectory.data_中
    trajectory.data_.emplace(index, data);
    // 返回新生成的IdType
    return IdType{trajectory_id, index};
  }

  // Returns an iterator to the element at 'id' or the end iterator if it does
  // not exist.
  // 获取指向id的ConstIterator格式的迭代器
  ConstIterator find(const IdType& id) const {
    return ConstIterator(*this, id);
  }

  // Inserts data (which must not exist already) into a trajectory.
  // 把id对应的data插入到表中
  void Insert(const IdType& id, const DataType& data) {
    CHECK_GE(id.trajectory_id, 0);
    CHECK_GE(GetIndex(id), 0);
    auto& trajectory = trajectories_[id.trajectory_id];
    trajectory.can_append_ = false;
    CHECK(trajectory.data_.emplace(GetIndex(id), data).second);
  }

  // Removes the data for 'id' which must exist.
  // 删除表中对应id的数据
  void Trim(const IdType& id) {
    // 获取这个id的trajectory id
    auto& trajectory = trajectories_.at(id.trajectory_id);
    // 找到指向这个数据的迭代器
    const auto it = trajectory.data_.find(GetIndex(id));
    CHECK(it != trajectory.data_.end()) << id;
    // 如果是最后一个数据, 就将轨迹设置成不可添加数据的状态
    if (std::next(it) == trajectory.data_.end()) {
      // We are removing the data with the highest index from this trajectory.
      // We assume that we will never append to it anymore. If we did, we would
      // have to make sure that gaps in indices are properly chosen to maintain
      // correct connectivity.
      trajectory.can_append_ = false;
    }
    // 删除这个数据
    trajectory.data_.erase(it);
    // 如果删除之后轨迹空了, 就把轨迹删除掉
    if (trajectory.data_.empty()) {
      trajectories_.erase(id.trajectory_id);
    }
  }

  // 对应id的数据是否存在
  bool Contains(const IdType& id) const {
    return trajectories_.count(id.trajectory_id) != 0 &&
           trajectories_.at(id.trajectory_id).data_.count(GetIndex(id)) != 0;
  }

  // 返回表中指定id对应的数据
  const DataType& at(const IdType& id) const {
    return trajectories_.at(id.trajectory_id).data_.at(GetIndex(id));
  }

  DataType& at(const IdType& id) {
    return trajectories_.at(id.trajectory_id).data_.at(GetIndex(id));
  }

  // Support querying by trajectory.
  // 获取这条轨迹第一个数据的迭代器
  ConstIterator BeginOfTrajectory(const int trajectory_id) const {
    return ConstIterator(*this, trajectory_id);
  }

  // 获取这条轨迹最后一个数据之后的迭代器, 也是下一条轨迹的开始位置
  ConstIterator EndOfTrajectory(const int trajectory_id) const {
    return BeginOfTrajectory(trajectory_id + 1);
  }

  // Returns 0 if 'trajectory_id' does not exist.
  // 指定轨迹内的数据的个数
  size_t SizeOfTrajectoryOrZero(const int trajectory_id) const {
    return trajectories_.count(trajectory_id)
               ? trajectories_.at(trajectory_id).data_.size()
               : 0;
  }

  // Returns count of all elements.
  // 所有轨迹的数据的个数和
  size_t size() const {
    size_t size = 0;
    for (const auto& item : trajectories_) {
      size += item.second.data_.size();
    }
    return size;
  }

  // Returns Range object for range-based loops over the nodes of a trajectory.
  // 获取这条轨迹的开始和结束的迭代器, 通过Range封装
  Range<ConstIterator> trajectory(const int trajectory_id) const {
    return Range<ConstIterator>(BeginOfTrajectory(trajectory_id),
                                EndOfTrajectory(trajectory_id));
  }

  // Returns Range object for range-based loops over the trajectory IDs.
  // 返回所有轨迹中的第一个id号与最后一个id号
  Range<ConstTrajectoryIterator> trajectory_ids() const {
    return Range<ConstTrajectoryIterator>(
        ConstTrajectoryIterator(trajectories_.begin()),
        ConstTrajectoryIterator(trajectories_.end()));
  }

  // 指向0号轨迹的第一个数据的迭代器
  ConstIterator begin() const { return BeginOfTrajectory(0); }
  // 整个表的最后位置的迭代器
  ConstIterator end() const {
    return BeginOfTrajectory(std::numeric_limits<int>::max());
  }
  // 整个表是否是空的
  bool empty() const { return begin() == end(); }

  // Returns an iterator to the first element in the container belonging to
  // trajectory 'trajectory_id' whose time is not considered to go before
  // 'time', or EndOfTrajectory(trajectory_id) if all keys are considered to go
  // before 'time'.
  // 返回这条轨迹上时间小于等于time的数据的迭代器
  ConstIterator lower_bound(const int trajectory_id,
                            const common::Time time) const {
    // 如果这个轨迹id的数据为0
    if (SizeOfTrajectoryOrZero(trajectory_id) == 0) {
      return EndOfTrajectory(trajectory_id);
    }

    const std::map<int, DataType>& trajectory =
        trajectories_.at(trajectory_id).data_;
    // 如果整个表的最后一个数据的时间比time小, 就返回EndOfTrajectory
    if (internal::GetTime(std::prev(trajectory.end())->second) < time) {
      return EndOfTrajectory(trajectory_id);
    }

    // 二分查找
    auto left = trajectory.begin();
    auto right = std::prev(trajectory.end());
    while (left != right) {
      // This is never 'right' which is important to guarantee progress.
      const int middle = left->first + (right->first - left->first) / 2;
      // This could be 'right' in the presence of gaps, so we need to use the
      // previous element in this case.
      auto lower_bound_middle = trajectory.lower_bound(middle);
      if (lower_bound_middle->first > middle) {
        CHECK(lower_bound_middle != left);
        lower_bound_middle = std::prev(lower_bound_middle);
      }
      if (internal::GetTime(lower_bound_middle->second) < time) {
        left = std::next(lower_bound_middle);
      } else {
        right = lower_bound_middle;
      }
    }

    return ConstIterator(*this, IdType{trajectory_id, left->first});
  }

 private:
 
  struct MapByIndex {
    bool can_append_ = true;
    // 这里的int指的是 NodeId或者SubmapId的index, DataType为实际存储的数据
    std::map<int, DataType> data_;
  };

  // c++11: static 修饰 成员函数, 代表静态成员函数
  // 静态成员函数是属于类的, 不是属于对象的. 静态成员函数只能访问静态成员变量

  // 只有 NodeId 和 SubmapId 才可以当做 MapById 的key
  static int GetIndex(const NodeId& id) { return id.node_index; }
  static int GetIndex(const SubmapId& id) { return id.submap_index; }

  // 所有的轨迹, 以及轨迹内的数据
  std::map<int, MapByIndex> trajectories_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_ID_H_

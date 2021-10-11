

## 第六章 代码总结与3D建图

### 6.1 代码的全面总结

### 6.2 Cartographer的优缺点分析

#### 6.2.1 优点

代码架构十分优美
各个模块独立性很强, 可以很方便的进行修改, 或则是单独拿出来做其他应用
代码鲁棒性非常高, 很少出现莫名崩掉的情况, 错误提示很好
代码命名非常规范, 能够清楚的根据变量名与函数名判断其代表的含义

总之, cartographer的代码十分值得学习与借鉴.

#### 6.2.2 缺点

**点云的预处理**
发生的拷贝次数太多
自适应体素滤波如果参数不好时计算量太大

**位姿推测器**

可能有问题的点

- 计算pose的线速度与角速度时, 是采用的数据队列开始和末尾的2个数据计算的
- 计算里程计的线速度与角速度时, 是采用的数据队列开始和末尾的2个数据计算的
- 使用里程计, 不使用imu时, **计算里程计的线速度方向**和**姿态的预测**时, 用的是里程计数据队列开始和末尾的2个数据的平均角速度计算的, **时间长了就不准**
- 不使用里程计, 不使用imu时, 用的是pose数据队列开始和末尾的2个数据的平均角速度计算的, **时间长了就不准**
- **添加位姿时, 没有用pose的姿态对imu_tracker_进行校准, 也没有对整体位姿预测器进行校准, 只计算了pose的线速度与角速度**
- 从代码上看, cartographer认为位姿推测器推测出来的位姿与姿态是准确的

可能的改进建议

- pose的距离越小, 匀速模型越能代替机器人的线速度与角速度, 计算pose的线速度与角速度时, 可以考虑使用最近的2个数据进行计算

- 里程计距离越短数据越准, 计算里程计的线速度与角速度时, 可以考虑使用最近的2个数据进行计算

- 使用里程计, 不使用imu时, 计算里程计的线速度方向时, 可以考虑使用里程计的角度进行计算

- 使用里程计, 不使用imu时, 进行姿态的预测时, 可以考虑使用里程计的角度进行预测

- 不使用里程计, 不使用imu时, 可以考虑用最近的2个pose计算线速度与角速度

- 使用pose对imu_tracker_的航向角进行校准

**基于Ceres的扫描匹配**

可能有问题的点

- 平移和旋转的残差项是逼近于先验位姿的, 当先验位姿不准确时会产生问题

可能的改进建议

- 先将地图的权重调大, 平移旋转的权重调小, 如 1000, 1, 1, 或者 100, 1, 1
- 调参没有作用的时候可以将平移和旋转的残差项注释掉

**后端优化**

优化时的计算量太大, 可以根据自己需求调整参数, 或者增加计算前的过滤.

在计算子图间约束的时候, 目前cartographer是根据节点个数来做的, 定位时又根据时间来决定是否进行全子图的匹配, 这部分计算的判断可以根据自己的需求增加一些, 以减少计算量.

### 6.3 TSDF地图

#### TSDF地图
TSDF算法简述
[https://zhuanlan.zhihu.com/p/390276710](https://zhuanlan.zhihu.com/p/390276710)

TSDF算法学习
[https://blog.csdn.net/zfjBIT/article/details/104648505](https://blog.csdn.net/zfjBIT/article/details/104648505)

#### TSDF地图与ProbabilityGrid地图的区别

TSDF2D类继承了Grid2D类

具体的栅格值保存在Grid2D里的correspondence_cost_cells_中, 只不过这里保存的不再是空闲的概率了. 而是tsd值转成的value.

```c++
ProbabilityGrid::ProbabilityGrid(const MapLimits& limits,
                                 ValueConversionTables* conversion_tables)
    : Grid2D(limits, kMinCorrespondenceCost, kMaxCorrespondenceCost,
             conversion_tables),
      conversion_tables_(conversion_tables) {}

/**
 * @brief 构造函数
 * 
 * @param[in] limits 地图坐标信息
 * @param[in] truncation_distance 0.3
 * @param[in] max_weight 10.0
 * @param[in] conversion_tables 转换表
 */
TSDF2D::TSDF2D(const MapLimits& limits, float truncation_distance,
               float max_weight, ValueConversionTables* conversion_tables)
    : Grid2D(limits, -truncation_distance, truncation_distance,
             conversion_tables),
      conversion_tables_(conversion_tables),
      value_converter_(absl::make_unique<TSDValueConverter>(
          truncation_distance, max_weight, conversion_tables_)),
      weight_cells_(
          limits.cell_limits().num_x_cells * limits.cell_limits().num_y_cells,
          value_converter_->getUnknownWeightValue()) {}
```

可以看到, ProbabilityGrid地图的栅格值的最大最小分别是 0.9 与 0.1, 而 TSDF地图的上遏制的最大最小分别是 0.3 与 -0.3.

TSDF地图保存tsd值的同时还保存了权重值, 权重值保存在TSDF2D类的weight_cells_中.

获取TSDF地图栅格值是通过TSDF2D::GetTSDAndWeight获取栅格值的, 同时获取到TSD值与权重值.

#### 栅格值更新的方式

新的权重 = 之前的weight + 新的weight
新的tsd值 = (之前的tsd值 * 之前的weight + 新的tsd值 * 新的weight) / (新的权重)

```c++
// TSDF地图栅格的更新, 分别更新tsd值与权重值
void TSDFRangeDataInserter2D::UpdateCell(const Eigen::Array2i& cell,
                                         float update_sdf, float update_weight,
                                         TSDF2D* tsdf) const {
  if (update_weight == 0.f) return;
  // 获取TSD值与权重值
  const std::pair<float, float> tsd_and_weight = tsdf->GetTSDAndWeight(cell);
  float updated_weight = tsd_and_weight.second + update_weight;
  float updated_sdf = (tsd_and_weight.first * tsd_and_weight.second +
                       update_sdf * update_weight) /
                      updated_weight;
  updated_weight =
      std::min(updated_weight, static_cast<float>(options_.maximum_weight()));
  tsdf->SetCell(cell, updated_sdf, updated_weight);
}
```

#### 相关性扫描匹配时使用TSDF计算得分
```c++
// 计算点云在指定像素坐标位置下与TSDF2D地图匹配的得分
float ComputeCandidateScore(const TSDF2D& tsdf,
                            const DiscreteScan2D& discrete_scan,
                            int x_index_offset, int y_index_offset) {
  float candidate_score = 0.f;
  float summed_weight = 0.f;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    const std::pair<float, float> tsd_and_weight =
        tsdf.GetTSDAndWeight(proposed_xy_index);
    const float normalized_tsd_score =
        (tsdf.GetMaxCorrespondenceCost() - std::abs(tsd_and_weight.first)) /
        tsdf.GetMaxCorrespondenceCost();
    const float weight = tsd_and_weight.second;
    candidate_score += normalized_tsd_score * weight;
    summed_weight += weight;
  }
  if (summed_weight == 0.f) return 0.f;
  candidate_score /= summed_weight;
  CHECK_GE(candidate_score, 0.f);
  return candidate_score;
}
```


#### TSDF地图的扫描匹配
InterpolatedTSDF2D
CreateTSDFMatchCostFunction2D

### 6.4 3D网格地图

HybridGridBase相当于Grid2D

HybridGrid相当于ProbabilityGrid

就是用三维网格替换了二维网格, 其余是差不多的.

和2D不同的是, 地图里保存的是odd, 而不是costodd, 即是占用的概率, 而不是miss的概率.

### 6.5 3D扫描匹配

#### LocalTrajectoryBuilder3D
点云的处理之前讲过了, 位姿预测器是一样的. 大部分是一样的, 不一样的点在下边.

#### RealTimeCorrelativeScanMatcher3D
首先分别对 xyz 与 绕xyz的旋转 这6个维度进行遍历, 生成所有的可能解
对所有的可能解进行打分, 选出最高分的解

#### CeresScanMatcher3D

基本上与2D是一样的, 只是地图这的残差变了.

OccupiedSpaceCostFunction3D
地图变成2个了, 一个高分辨率地图, 一个低分辨率地图.

地图残差的计算基本也是一样的, 就是拿点云对应的栅格值当做残差, 只不过作为残差的是(1. - probability).

InterpolatedGrid
手动实现了双三次插值

#### 将点云插入到三维网格地图里

旋转直方图
首先计算了点云的旋转直方图, 子图的旋转直方图就是点云的旋转直方图的累加.

表征点云与地图的角度特征.

**推荐2篇文章**
- cartographer 3D scan matching 理解
[https://www.cnblogs.com/mafuqiang/p/10885616.html](https://www.cnblogs.com/mafuqiang/p/10885616.html)

- Cartographer源码阅读3D-Submap创建 
[https://blog.csdn.net/yeluohanchan/article/details/109462508?spm=1001.2014.3001.5501](https://blog.csdn.net/yeluohanchan/article/details/109462508?spm=1001.2014.3001.5501)


插入器 RangeDataInserter3D

将点云分别插入到低分辨率地图与高分辨率地图中.

### 6.6 3D后端优化

#### PoseGraph3D

基本一样, 只不过位姿是6维的了, 不需要再去与重力对齐向量相乘了, 直接获取local_pose, 不用进行旋转变换了.

#### ConstraintBuilder3D
基本一样, 只不过调用的是FastCorrelativeScanMatcher3D.

#### FastCorrelativeScanMatcher3D
**构造**
构造的时候需要传入高分辨率地图与低分辨率地图, 以及地图对应的旋转直方图. 
之后将高分辨率地图弄成多分辨率地图, 保存低分辨率地图与旋转直方图.
构造时生成了一个低分辨率地图的匹配器.

**匹配**
在离散点云的时候通过旋转直方图匹配滤掉了一部分候选scan.

**分枝定界算法**
每一个候选解生成8个候选解, 3个方向.
第0层的时候将叶节点与低分辨率地图再进行匹配一下, 得分大于阈值才能返回.

#### OptimizationProblem3D

根据参数选择是否对节点与子图的pose的z坐标进行优化

优化时第一个子图固定了xyz, 旋转固定了yaw, 只优化绕xy的旋转, 因为绕xy的旋转可以通过重力的方向进行约束.

由于旋转是通过四元数表示, 所以在ceres中添加了QuaternionParameterization, 以对四元数进行更新.

多了根据imu计算的残差, 分为加速度的残差与旋转的残差

其余的残差基本一样.

## 第七章 地图保存与纯定位模式

### 7.1 Submap与ROS格式地图间的格式转换

### 7.2 ROS地图的发布

### 7.3 纯定位模式


```c++
// node_main.cc
if (!FLAGS_load_state_filename.empty()) {
  node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
}

// node.cc
void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_.LoadState(state_filename, load_frozen_state);
  load_state_ = true;
}

// map_builder_bridge.cc
void MapBuilderBridge::LoadState(const std::string& state_filename, bool load_frozen_state) {
  // Check if suffix of the state file is ".pbstream".
  const std::string suffix = ".pbstream";
  // 检查后缀是否是.pbstream
  CHECK_EQ(state_filename.substr(
               std::max<int>(state_filename.size() - suffix.size(), 0)),
           suffix)
      << "The file containing the state to be loaded must be a "
         ".pbstream file.";
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  // 加载文件内容
  cartographer::io::ProtoStreamReader stream(state_filename);
  // 解析数据
  map_builder_->LoadState(&stream, load_frozen_state);
}

// map_builder.cc
// 构造时候初始化pose_graph_, 之后在重定位时候才能向pose_graph_中添加数据
std::map<int, int> MapBuilder::LoadState(
    io::ProtoStreamReaderInterface* const reader, bool load_frozen_state) {
  // 从文件中添加轨迹
  // 更新约束中节点与子图的轨迹id
  // 从获取到的位姿图中生成submap_poses
  // 从获取到的位姿图中生成node_poses
  // 将landmark_poses添加到位姿图中
  // 向pose_graph_中添加信息
  // 添加子图的附属的节点
}

// node.cc
if (FLAGS_start_trajectory_with_default_topics) {
  node.StartTrajectoryWithDefaultTopics(trajectory_options);
}

// pose_graph_2d.cc 优化之后进行子图的裁剪
{
  TrimmingHandle trimming_handle(this);
  // 进行子图的裁剪, 如果没有裁剪器就不裁剪
  for (auto& trimmer : trimmers_) {
    trimmer->Trim(&trimming_handle); // PureLocalizationTrimmer::Trim()
  }
  // 如果裁剪器处于完成状态, 就把裁剪器删除掉
  trimmers_.erase(
      // c++11: std::remove_if 如果回调函数函数返回真,则将当前所指向的参数移到尾部,返回值是被移动区域的首个元素
      std::remove_if(trimmers_.begin(), trimmers_.end(),
                      [](std::unique_ptr<PoseGraphTrimmer>& trimmer) {
                        return trimmer->IsFinished(); // 调用PureLocalizationTrimmer::IsFinished()
                      }),
      trimmers_.end());
}

// 裁剪子图与节点, 及相关的约束
void PoseGraph2D::TrimmingHandle::TrimSubmap(const SubmapId& submap_id) {
  // 获取除submap_id外的所有子图的所有节点的id(nodes_to_retain), 这些节点是需要保留的
  // 找到在submap_id的子图内部同时不在别的子图内的节点(nodes_to_remove), 这些节点需要删除
  // 删除submap_id相关的约束
  // 删除与nodes_to_remove中节点相关联的约束, 并对对应的submap_id进行标记
  // 对标记的submap_id进行检查, 看是否还存在其他子图间约束
  // 删除没有子图间约束的标记的子图的扫描匹配器
  // 删除submap_id这个子图的指针
  // 删除submap_id这个子图的匹配器, 与多分辨率地图
  // 删除optimization_problem_中的submap_id这个子图
  // 删除nodes_to_remove中的节点
}
```




##
4.4 生成3D局部地图
  a. 查找表的实现
  b. 3D网格地图的实现
  c. 3D网格地图的更新方式
  d. 如何将点云插入到3D网格地图中
4.5 3D情况下的基于局部地图的扫描匹配
  a. 基于IMU与里程计的先验位姿估计
  b. 将点云分别进行高分辨率与低分辨率的体素滤波
  c. 使用实时的相关性扫描匹配对高分辨率体素滤波后的点云进行粗匹配
  d. 进行基于图优化的扫描匹配实现精匹配
  e. 旋转直方图的作用
  f. 基于ceres-solver的优化模型搭建
5.3 3D情况下的后端优化
  a. 基于ceres-solver的后端位姿图优化模型的搭建
  b. 向位姿图中添加基于GPS的2个连续位姿间的约束
  c. 向位姿图中添加基于IMU的角速度和线性加速度的约束
  d. 残差项雅克比矩阵的计算与优化模型的求解
6.3 3D情况下的回环检测
  a. 多分辨率网格地图的生成
  b. 计算点云在低分辨率地图下的得分作为初值
  c. 通过旋转直方图对点云所有可能的旋转角度进行评分
  d. 根据评分是否大于阈值生成按照可能角度旋转后的点云
  e. 生成最低分辨率地图下的所有的可能解
  f. 对所有的可能解进行评分与排序
  g. 使用分支定界算法找到最优解
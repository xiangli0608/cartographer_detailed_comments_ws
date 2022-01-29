

### 4.3 扫描匹配的概念与实现


### 4.4 相关性扫描匹配 








### 4.5 Ceres简介



###  4.6 基于Ceres的扫描匹配


## 第五章 后端优化

### 5.1 什么是后端优化

#### 什么是图

由**节点**和**边**组成的一种数据结构, 节点之间的关系可以是任意的, 图中任意两节点之间都可能相关(存在边).

#### 什么是位姿图

SPA论文中对位姿图的定义: 位姿图是一组通过非线性约束连接的机器人位姿, 这些非线性约束是从对附近位姿共有的特征的观察中获得的.

位姿图是一种图, 节点代表位姿, 边代表2个位姿间的相对坐标变换(也叫约束).

![image-20210917210027512](C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210917210027512.png)

三角形表示机器人位姿, 三角形之间的连线表示约束(坐标变换)



#### 什么是优化

由于前端里程计会有累计误差, 那有没有一种方法可以将这种累计误差减小甚至消除掉呢?

这就是优化的目的与作用. 

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210915211330019.png" alt="image-20210915211330019" style="zoom: 50%;" />

​																			优化前

![image-20210915211340818](C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210915211340818.png)

​																			优化后



SPA论文中的定义: 其目标是寻找一个节点间的配置关系, 使得节点间约束的测量概率最大.

通过图结构, 根据约束与所有的节点, 将所有的误差减小, 并且平均分散到每个节点上去, 以达到减小整体误差的作用.

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210915212140605.png" alt="image-20210915212140605" style="zoom: 80%;" />

优化的这部分模块, 一般称为后端部分, 这就是 **后端优化** 模块的由来.



#### 如何做优化呢

**SPA优化**

论文存放在 cartographer_detailed_comments_ws/src/cartographer/docs 文件夹内

论文名字为: 2010 - Efficient Sparse Pose Adjustment for 2D Mapping - Konolige et al.pdf

推荐自己阅读一下 前第3节的内容以及第4节的AB两部分

论文翻译 [https://blog.csdn.net/u014527548/article/details/106238658](https://blog.csdn.net/u014527548/article/details/106238658)



**残差方程**

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210915211733956.png" alt="image-20210915211733956" style="zoom: 67%;" />



**目标**

- 第一步: 确定2个节点在global坐标系下的相对位姿变换
- 第二步: 通过其他方式再次获取这2个节点的相对位姿变换
- 第三步: 对这2个相对位姿变换的差 的最小二乘问题进行求解 
- 第四步: 进行求解之后会得到一个增量  $\Delta{x}$  , 将当前位姿加上这个增量后就得到了优化后的位姿

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210915211814565.png" alt="image-20210915211814565" style="zoom:67%;" />



#### cartographer代码里实现位姿图优化的思路

cartorgapher使用ceres进行位姿图优化, ceres求解的是残差的和. 

单一约束是不能进行优化的, 至少要有2个约束才能进行基于ceres的位姿图优化.



所以现在问题就是如何找到2个约束

第一个约束, 根据SPA论文, 就是节点global坐标系下的相对坐标变换.

所以, 第二个约束怎么求呢?



cartographer中设置了如下5种第二个约束及残差, 共同进行优化.

- 第一种残差 将节点(tracking的位姿)与节点(子图原点位姿)在global坐标系下的相对位姿 与 约束 的差值作为残差项

- 第二种残差 landmark数据 与 通过2个节点位姿插值出来的相对位姿 的差值作为残差项

- 第三种残差 节点与节点间在global坐标系下的相对坐标变换 与 通过里程计数据插值出的相对坐标变换 的差值作为残差项

- 第四种残差 节点与节点间在global坐标系下的相对坐标变换 与 相邻2个节点在local坐标系下的相对坐标变换 的差值作为残差项

- 第五种残差 节点与gps坐标系原点在global坐标系下的相对坐标变换 与 通过gps数据进行插值得到的相对坐标变换 的差值作为残差项



所以, 整个后端优化问题, 可以简化成

- 求节点在global坐标系下的位姿 (节点)

- 求节点的约束(子图内约束与子图间约束, 子图间约束又称为回环约束) (边) 

- 求其他四种残差共同构建残差方差

- 进行优化求解



节点在global坐标系下的位姿与子图内约束

- PoseGraph2D::ComputeConstraintsForNode

子图间约束又称为回环约束

- ConstraintBuilder2D
- FastCorrelativeScanMatcher2D
- PrecomputationGridStack2D

构建残差方差与优化求解

- OptimizationProblem2D::Solve



### 5.2 重点函数讲解

#### 节点与约束的概念

节点: tracking_frame的位姿, 子图原点的位姿

约束: tracking_frame与子图原点间的坐标变换

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210917210622636.png" alt="image-20210917210622636" style="zoom: 67%;" />



#### ComputeLocalToGlobalTransform

这里更正一下: 视频里讲这个函数时说的是 后端优化的第一个节点的位姿是(0, 0, 0),  是错误的, 应该是

**后端优化的global到local间的坐标变换, 如果没提前设置的话就是平移0与旋转0**.



**下边的函数, 如果是后端的第一个节点, GetLocalToGlobalTransform会返回(0, 0, 0), 然后还要乘以这个节点再local坐标系下的pose, 乘的结果才是global坐标系下第一个节点的位姿.**

```c++
NodeId PoseGraph2D::AddNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps) {
  
  // GetLocalToGlobalTransform
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);
```



```c++
transform::Rigid3d PoseGraph2D::ComputeLocalToGlobalTransform(
    const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
    const int trajectory_id) const {
  auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
  auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);
  // 没找到这个轨迹id
  if (begin_it == end_it) {
    const auto it = data_.initial_trajectory_poses.find(trajectory_id);
    // 如果设置了初始位姿
    if (it != data_.initial_trajectory_poses.end()) {
      return GetInterpolatedGlobalTrajectoryPose(it->second.to_trajectory_id,
                                                 it->second.time) *
             it->second.relative_pose;
    }
    // note: 没设置初始位姿就将返回(0,0,0)的平移和旋转
    else {
      return transform::Rigid3d::Identity();
    }
  }

  // 找到了就获取优化后的最后一个子图的id
  const SubmapId last_optimized_submap_id = std::prev(end_it)->id;
  // Accessing 'local_pose' in Submap is okay, since the member is const.
  // 通过最后一个优化后的 global_pose * local_pose().inverse() 获取 global_pose->local_pose的坐标变换
  // tag: 画图说明一下
  return transform::Embed3D(
             global_submap_poses.at(last_optimized_submap_id).global_pose) *
         data_.submap_data.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}
```

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210918212549742.png" alt="image-20210918212549742" style="zoom:50%;" />



#### 线性差值公式推导

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/线性差值.jpg" alt="线性差值" style="zoom: 67%;" />

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/线性差值2.jpg" alt="线性差值2" style="zoom: 50%;" />



#### 第一次看到的子图的指针进行保存

data_.submap_data: 保存子图的指针

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210919144703820.png" alt="image-20210919144703820" style="zoom:67%;" />



#### 线程池

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210919144539528.png" alt="image-20210919144539528" style="zoom: 67%;" />

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210919144606100.png" alt="image-20210919144606100" style="zoom: 67%;" />



#### ComputeConstraintsForNode




##### InitializeGlobalSubmapPoses

data_.global_submap_poses_2d: 全都是优化后的子图在global坐标系下的pose

optimization_problem_->submap_data(): 包含了优化后和还没有进行优化的 子图在global坐标系下的pose

ComputeLocalToGlobalTransform()这个函数的参数, 始终都是data_.global_submap_poses_2d, 计算的是优化后的global指向local的坐标变换.

```c++
transform::Rigid3d PoseGraph2D::ComputeLocalToGlobalTransform(
    const MapById<SubmapId, optimization::SubmapSpec2D>& global_submap_poses,
    const int trajectory_id) const {
  auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
  auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);
  const SubmapId last_optimized_submap_id = std::prev(end_it)->id;
  return transform::Embed3D(
             global_submap_poses.at(last_optimized_submap_id).global_pose) *
         data_.submap_data.at(last_optimized_submap_id)
             .submap->local_pose()
             .inverse();
}

// 1 如果只有1个子图 
optimization_problem_->AddSubmap(
    trajectory_id, transform::Project2D(
        ComputeLocalToGlobalTransform( // 会返回(0, 0,0)
            data_.global_submap_poses_2d, trajectory_id) *
        insertion_submaps[0]->local_pose()));

// 2 有2个子图, 但是第二个子图没保存位姿的情况
const auto& first_submap_pose = submap_data.at(last_submap_id).global_pose;      
optimization_problem_->AddSubmap(
    trajectory_id,
    first_submap_pose *
        constraints::ComputeSubmapPose(*insertion_submaps[0]).inverse() *
        constraints::ComputeSubmapPose(*insertion_submaps[1]));
```



##### 计算子图内约束

```c++
const transform::Rigid2d local_pose_2d =
    transform::Project2D(constant_data->local_pose * // 三维转平面
                         transform::Rigid3d::Rotation(
                             constant_data->gravity_alignment.inverse()));

const transform::Rigid2d global_pose_2d =
    optimization_problem_->submap_data().at(matching_id).global_pose *
    constraints::ComputeSubmapPose(*insertion_submaps.front()).inverse() *
    local_pose_2d;

const transform::Rigid2d constraint_transform =
          constraints::ComputeSubmapPose(*insertion_submaps[i]).inverse() *
          local_pose_2d;
```

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210920101755031.png" alt="image-20210920101755031" style="zoom:50%;" />

#####  子图间约束

与子图内约束是一样的, 也是在local坐标系下, submap的坐标原点指向tracking_frame的坐标变换




### 5.3 基于分支定界算法的扫描匹配

#### 滑动窗口法生成多分辨率地图

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210923210416179.png" alt="image-20210923210416179" style="zoom: 67%;" />



#### 分支定界算法

- **分支**: 对当前层候选解进行分支(扩充), 生成下一层分辨率地图上的4个候选解

- **排序**: 对下一层分辨率地图上的4个候选解进行打分并**降序排序**

- **定界**: 将当前层的最高得分, 当做下一次分支定界算法的分数阈值

- **剪枝**: 只要当前层的候选解的得分, 有小于传入的阈值的, 就break, 因为是排好序的

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210925192955844.png" alt="image-20210925192955844" style="zoom: 67%;" />





### 5.4 优化问题的构建与求解

#### 5.4.1 向优化问题中添加数据

##### 添加节点数据

```c++
optimization_problem_->AddTrajectoryNode(
    matching_id.trajectory_id,
    optimization::NodeSpec2D{constant_data->time, local_pose_2d,
                             global_pose_2d,
                             constant_data->gravity_alignment});
```

##### 添加子图坐标原点数据

```c++
optimization_problem_->AddSubmap(
    trajectory_id,
    first_submap_pose *
        constraints::ComputeSubmapPose(*insertion_submaps[0]).inverse() *
        constraints::ComputeSubmapPose(*insertion_submaps[1]));
```

##### 添加其他传感器数据

```c++
optimization_problem_->AddImuData(trajectory_id, imu_data);
optimization_problem_->AddOdometryData(trajectory_id, odometry_data);
optimization_problem_->AddFixedFramePoseData(trajectory_id, fixed_frame_pose_data);
```

#### 5.4.2  几种计算相对位姿的方式

##### 节点在global坐标系下的位姿

```c++
const transform::Rigid3d optimized_pose(
  GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);
```

```c++
ComputeLocalToGlobalTransform() // 传入的始终是data_.global_submap_poses_2d 
data_.global_submap_poses_2d  // 只有在第一次优化完之后才有数据
```


节点在global坐标系下的第一帧的位姿, 就是这个节点在local坐标系下的位姿.

之后的节点在global坐标系下的位姿 是通过 global到local的坐标变换乘以local坐标系下的位姿得到的.

##### submap在global坐标系下的位姿

```c++
optimization_problem_->AddSubmap(
  trajectory_id, transform::Project2D(
​            ComputeLocalToGlobalTransform(
​              data_.global_submap_poses_2d, trajectory_id) *
​            insertion_submaps[0]->local_pose()));

optimization_problem_->AddSubmap(
  trajectory_id,
  first_submap_pose *
​    constraints::ComputeSubmapPose(*insertion_submaps[0]).inverse() *
​    constraints::ComputeSubmapPose(*insertion_submaps[1]));
```

子图在global坐标系下的第一帧的位姿,就是这个子图在local坐标系下的位姿.

之后的子图在global坐标系下的位姿 是通过 第一个子图在global坐标系下的pose 乘以 第一个子图到第二个子图在local坐标系下的位姿变换 得到的.


##### 子图内约束

local坐标系下, 子图原点指向tracking_frame的坐标变换

```c++
const transform::Rigid2d local_pose_2d =
  transform::Project2D(constant_data->local_pose * // 三维转平面
  transform::Rigid3d::Rotation(
    constant_data->gravity_alignment.inverse()));

// 计算 子图原点 指向 node坐标 间的坐标变换(子图内约束)
const transform::Rigid2d constraint_transform =
  constraints::ComputeSubmapPose(*insertion_submaps[i]).inverse() *
  local_pose_2d;
```



##### 子图间约束

先通过子图在global坐标系下的坐标的逆, 乘以节点在global坐标系下的坐标, 获取子图原点在glboal坐标系下指向节点的相对坐标变换.

然后根据子图在local坐标系下的位姿乘以这个坐标变换, 得到节点在local坐标系下的预测位姿, 在通过分枝定界粗匹配与ceres的精匹配, 对这个节点位姿进行校准, 校准后的位姿还是local坐标系下的.

最后, 通过子图在local坐标系下位姿的逆, 乘以这个节点校准后的位姿, 得到子图间约束, 是local坐标系下, 子图原点指向节点的相对坐标变换.



```c++
const transform::Rigid2d initial_relative_pose =
  optimization_problem_->submap_data().at(submap_id).global_pose.inverse() *
  optimization_problem_->node_data().at(node_id).global_pose_2d;

const transform::Rigid2d initial_pose =
  ComputeSubmapPose(*submap) * initial_relative_pose;

const transform::Rigid2d constraint_transform =
  ComputeSubmapPose(*submap).inverse() * pose_estimate;
```



##### 4种计算相对位姿的方式总结

1. 节点 通过`GetLocalToGlobalTransform * constant_data->local_pose`进行global下位姿的计算
2. 子图 通过对前一个子图到后一个子图的坐标变换进行累计, 得到子图在global坐标系下的位姿
3. 子图内约束 local坐标系系下, 子图原点指向节点间的坐标变换
4. 子图间约束 根据global坐标计算初值, 然后通过分支定界算法粗匹配与ceres的精匹配, 获取校准后的位姿, 最后计算local坐标系系下, 子图原点指向校准后的节点间的坐标变换



#### 5.4.3 残差项的构建

##### 第一种残差 

将节点与子图原点在global坐标系下的相对位姿 与 约束 的差值作为残差项

- 第一种坐标变换: 节点与子图原点在global坐标系下的坐标变换
- 第二种坐标变换: 子图内约束与子图间约束

##### 第二种残差 

landmark数据 与 通过2个节点位姿插值出来的相对位姿 的差值作为残差项

- 第一种坐标变换: landmark数据的时间在2个节点位姿中插值出来的位姿
- 第二种坐标变换: landmark数据中的landmark_to_tracking_transform_


```c++
// 第一种坐标变换
// start: 
    const std::tuple<std::array<T, 4>, std::array<T, 3>>
        interpolated_rotation_and_translation = InterpolateNodes2D(
            prev_node_pose, prev_node_gravity_alignment_, next_node_pose,
            next_node_gravity_alignment_, interpolation_parameter_);
// end:
  const std::tuple<std::array<double, 4>, std::array<double, 3>>
      rotation_and_translation =
          InterpolateNodes2D(prev_node_pose.data(), prev_node.gravity_alignment,
                             next_node_pose.data(), next_node.gravity_alignment,
                             interpolation_parameter);
  // 将landmark的数据从tracking_frame下的位姿转到global坐标系下
  return transform::Rigid3d::FromArrays(std::get<0>(rotation_and_translation),
                                        std::get<1>(rotation_and_translation)) *
         observation.landmark_to_tracking_transform;
// 第二种坐标变换
landmark_to_tracking_transform_
    
    
// 在初始时刻这2个约束计算的结果是一样的, 直到
// landmark_node.second.global_landmark_pose.has_value 为 true 时
    
const transform::Rigid3d starting_point =
    andmark_node.second.global_landmark_pose.has_value()
        ? landmark_node.second.global_landmark_pose.value()
        : GetInitialLandmarkPose(observation, prev->data, next->data,
            *prev_node_pose, *next_node_pose);
  
// 更新data_.landmark_nodes
for (const auto& landmark : optimization_problem_->landmark_data()) {
    data_.landmark_nodes[landmark.first].global_landmark_pose = landmark.second;
}
```


##### 第三种残差 

节点与节点间在global坐标系下的相对坐标变换 与 通过里程计数据插值出的相对坐标变换 的差值作为残差项

- 第一种坐标变换: 相邻2个节间在global坐标系下的坐标变换

- 第二种坐标变换: 再分别计算这2个节点的时间在里程计数据队列中插值得到的2个里程计位姿, 计算这2个里程计位姿间的坐标变换

  <img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20211007174744091.png" alt="image-20211007174744091" style="zoom: 67%;" />

##### 第四种残差

节点与节点间在global坐标系下的相对坐标变换 与 相邻2个节点在local坐标系下的相对坐标变换 的差值作为残差项

- 第一种坐标变换: 相邻2个节间在global坐标系下的坐标变换
- 第二种坐标变换: 相邻2个节点在local坐标系下的坐标变换

##### 第五种残差 

节点与gps坐标系原点在global坐标系下的相对坐标变换 与 通过gps数据进行插值得到的相对坐标变换 的差值作为残差项

- 第一种坐标变换: 节点对应的时刻在gps数据中插值得到的gps相对于gps坐标系原点的位姿
- 第二种坐标变换: 节点在global坐标系下 与 gps坐标系原点 的相对坐标变换

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210927215138987.png" alt="image-20210927215138987" style="zoom: 67%;" />



### 5.5 优化后

#### 5.5.1 优化结果的保存

```c++
  for (const auto& C_submap_id_data : C_submaps) {
    submap_data_.at(C_submap_id_data.id).global_pose =
        ToPose(C_submap_id_data.data);
  }
  for (const auto& C_node_id_data : C_nodes) {
    node_data_.at(C_node_id_data.id).global_pose_2d =
        ToPose(C_node_id_data.data);
  }
  for (const auto& C_fixed_frame : C_fixed_frames) {
    trajectory_data_.at(C_fixed_frame.first).fixed_frame_origin_in_map =
        transform::Embed3D(ToPose(C_fixed_frame.second));
  }
  for (const auto& C_landmark : C_landmarks) {
    landmark_data_[C_landmark.first] = C_landmark.second.ToRigid();
  }
```



#### 5.5.2 优化前后的不变量与变化量

##### 不变量

- 第一帧子图在global坐标系下的位姿不会被优化所改变

```c++
problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
```

由于优化前后第一个子图原点在global坐标系下的位姿是不会发生变化的, 所以global坐标系是不会改变的

- 冻结轨迹的 节点pose, 子图pose, landmark的pose 是不变的

##### 变化量

- 除第一个子图外的其他子图的global坐标系下的pose
- 所有节点在global坐标系下的pose
- 所有landmark数据在global坐标系下的pose
- gps第一帧数据在global坐标系下的pose



#### 5.5.3 优化的贡献

- 第一个贡献 优化了位姿: 子图, 节点, landmark, gps第一帧坐标

- 第二个贡献 预测了没有优化的节点位姿

- 第三个贡献 ComputeLocalToGlobalTransform

```c++
NodeId PoseGraph2D::AddNode(
    std::shared_ptr<const TrajectoryNode::Data> constant_data,
    const int trajectory_id,
    const std::vector<std::shared_ptr<const Submap2D>>& insertion_submaps) {
  // 将节点在local坐标系下的坐标转成global坐标系下的坐标
  const transform::Rigid3d optimized_pose(
      GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);
```

ComputeLocalToGlobalTransform() 函数是计算最后一个子图原点的global坐标指向local坐标的坐标变换, 为什么要是最后一个子图的坐标呢?

local坐标系下的位姿都是相对与前一个节点的相对位姿, 时间越短, 相对位姿越准. 

优化后的最后一个子图位姿, 与当前节点的位姿在时间上距离最近. 

所以, 最后的一个优化后的子图位姿的相对坐标变换, 在时间上最接近当前节点与当前子图, 时间越短, 乘以相对坐标变换后得到的节点或者子图在global坐标系下的位姿越接近真实值.

- 第四个贡献 MapBuilderBridge

```c++
MapBuilderBridge::GetSubmapList()
    map_builder_->pose_graph()->GetAllSubmapPoses();

MapBuilderBridge::GetLocalTrajectoryData()
	map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id),

MapBuilderBridge::HandleTrajectoryQuery
	map_builder_->pose_graph()->GetTrajectoryNodePoses()
    
MapBuilderBridge::GetTrajectoryNodeList()
    map_builder_->pose_graph()->GetTrajectoryNodePoses()
    map_builder_->pose_graph()->constraints()

MapBuilderBridge::GetLandmarkPosesList()
	map_builder_->pose_graph()->GetLandmarkPoses()

MapBuilderBridge::GetConstraintList()
    pose_graph()->GetTrajectoryNodePoses()
    map_builder_->pose_graph()->GetAllSubmapPoses()
    map_builder_->pose_graph()->constraints()
```



所以, 后端优化后的结果是与前端无关的.

这也解释了为什么carto官方文档说的是前端坐标系是不会变化的, 就是由于后端的结果不会作用在前端上.




## 第六章 代码总结与3D建图

### 6.1 视频的全面总结

Node类使用前端数据的函数

```c++
Node::PublishLocalTrajectoryData // 发布点云与tf
```

Node类使用后端数据的函数

```c++
Node::PublishLocalTrajectoryData 	// 发布点云与tf
Node::PublishTrajectoryNodeList() 	// 可视化所有的节点位姿
Node::PublishLandmarkPosesList()	// 可视化所有的landmark位姿
Node::PublishConstraintList			// 可视化所有的约束
```



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

推荐2篇文章

- TSDF算法简述
[https://zhuanlan.zhihu.com/p/390276710](https://zhuanlan.zhihu.com/p/390276710)
- TSDF算法学习

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

构造时

**匹配**

生成了一个低分辨率地图的匹配器.

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

### 7.1 地图格式的转换

### 7.2 地图的发布

### 7.3 纯定位模式

```c++

// node_main.cc
if (!FLAGS_load_state_filename.empty()) {
 node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
}

// node.cc
void Node::LoadState(const std::string& state_filename,
​           const bool load_frozen_state) {
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
​        std::max<int>(state_filename.size() - suffix.size(), 0)),
​      suffix)
   << "The file containing the state to be loaded must be a "
​     ".pbstream file.";
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
​           [](std::unique_ptr<PoseGraphTrimmer>& trimmer) {
​            return trimmer->IsFinished(); // 调用PureLocalizationTrimmer::IsFinished()
​           }),
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



## 第八章 调参总结与工程化建议

### 8.1 调参总结
#### 8.1.1 降低延迟与减小计算量

**前端部分**
- 减小 max_range, 减小了需要处理的点数, 在雷达数据远距离的点不准时一定要减小这个值

- 增大 voxel_filter_size, 相当于减小了需要处理的点数

- 增大 submaps.resolution, 相当于减小了匹配时的搜索量

- 对于自适应体素滤波 减小 min_num_points与max_range, 增大 max_length, 相当于减小了需要处理的点数


**后端部分**

- 减小 optimize_every_n_nodes, 降低优化频率, 减小了计算量

- 增大 MAP_BUILDER.num_background_threads, 增加计算速度

- 减小 global_sampling_ratio, 减小计算全局约束的频率

- 减小 constraint_builder.sampling_ratio, 减少了约束的数量

- 增大 constraint_builder.min_score, 减少了约束的数量

- 减小分枝定界搜索窗的大小, 包括linear_xy_search_window,inear_z_search_window, angular_search_window

- 增大 global_constraint_search_after_n_seconds, 减小计算全局约束的频率

- 减小 max_num_iterations, 减小迭代次数

#### 8.1.2 降低内存

增大子图的分辨率 submaps.resolution

#### 8.1.3 常调的参数
```lua
 TRAJECTORY_BUILDER_2D.min_range = 0.3
 TRAJECTORY_BUILDER_2D.max_range = 100.
 TRAJECTORY_BUILDER_2D.min_z = 0.2 -- / -0.8
 TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02

 TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.
 TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
 TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.

 TRAJECTORY_BUILDER_2D.submaps.num_range_data = 80.
 TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1 -- / 0.02

 POSE_GRAPH.optimize_every_n_nodes = 160. -- 2倍的num_range_data以上
 POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
 POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
 POSE_GRAPH.constraint_builder.min_score = 0.48
 POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
```



###  8.2 工程化建议

#### 8.2.1 工程化的目的

根据机器人的**传感器硬件**, 最终能够实现**稳定地**构建一张**不叠图**的二维栅格地图.

由于cartographer代码十分的优秀, 所以cartographer的稳定性目前已经很好了, 比目前的大部分slam的代码都稳定, 很少会出现崩掉的情况, 最多就是会由于某些原因提示错误.

#### 8.2.2 如何提升建图质量

最简单的一种方式, 选择好的传感器. 选择频率高(25hz以上), 精度高的雷达, 精度高的imu, 这样的传感器配置下很难有建不好的地图.

##### 如果只能用频率低的雷达呢


由于频率低时的叠图基本都是在旋转时产生的, 所以推荐使用一个好的imu, 然后建图的时候让机器人的**移动与旋转速度慢一点**(建图轨迹与建图速度十分影响建图效果), 这时候再看建图效果.

如果效果还不行, 调ceres的匹配权重, 将地图权重调大, 平移旋转权重调小. 

如果效果还不行, 可以将代码中平移和旋转的残差注释掉.

如果效果还不行, 那就得改代码了, 去改位姿推测器那部分的代码, 让预测的准一点.

##### 里程计

为什么一直没有说里程计, 就是由于cartographer中对里程计的使用不太好.

cartographer中对里程计的使用有2部分, 一个是前端的位姿推测器, 一个是后端根据里程计数据计算残差. 后端部分的使用是没有问题的.

如果想要在cartographer中使用里程计达到比较好的效果, 前端的位姿推测器这部分需要自己重写. 

可以将karto与gmapping的使用里程计进行预测的部分拿过来进行使用, 改完了之后就能够达到比较好的位姿预测效果了.

##### 粗匹配

cartographer的扫描匹配中的粗匹配是一种暴力匹配的方法, 目的是对位姿预测出的位姿进行校准, 但是这个扫描匹配的计算量太大了, 导致不太好用.

这块可以进行改进, 可以将karto的扫描匹配的粗匹配放过来, karto的扫描匹配的计算量很小, 当做粗匹配很不错.

##### 地图

有时前端部分生成的地图出现了叠图, 而前端建的地图在后端是不会被修改的, 后端优化只会优化节点位姿与子图位姿.

同时cartographer_ros最终生成的地图是将所有地图叠加起来的, 就会导致这个叠图始终都存在, 又或者是后边的地图的空白部分将前边的地图的边给覆盖住了, 导致墙的黑边消失了.

后端优化会将节点与子图的位姿进行优化, 但是不会改动地图, 所以可以在最终生成地图的时候使用后端优化后的节点重新生成一次地图, 这样生成的地图的效果会比前端地图的叠加要好很多.

这块的实现可以参考一下我写的实时生成三维点云地图部分的代码.

##### 更极致的修改

后端优化后的节点与子图位姿是不会对前端产生影响的, 这块可以进行优化一下, 就是前端匹配的时候, 不再使用前端生成的地图进行匹配, 而是使用后端生成的地图进行匹配, 这样就可以将后端优化后的效果带给前端. 但是这要对代码进行大改, 比较费劲.


#### 8.2.3 降低计算量与内存

- 体素滤波与自适应体素滤波的计算量(不是很大)

- 后端进行子图间约束时的计算量很大

- 分支定界算法的计算量很大
- 降低内存, 内存的占用基本就是多分辨率地图这, 每个子图的多分辨率地图都进行保存是否有必要

#### 8.2.4 纯定位的改进建议

目前cartographer的纯定位和正常的建图是一样的, 只是仅保存3个子图, 依然要进行后端优化.


这就导致了几个问题:

第一个: 前端的扫描匹配, 是当前的雷达点云与当前轨迹的地图进行匹配, 而不是和之前的地图进行匹配, 这就导致了定位时机器人当前的点云与之前的地图不一定能匹配很好, 就是因为当前的点云是匹配当前轨迹的地图的, 不是与之前的地图进行匹配.

第二个: 纯定位其实就是建图, 所以依然会进行回环检测与后端优化, 而后端优化的计算在定位这是没有必要的, 带来了额外的计算量.

第三个: 纯定位依然会进行回环检测, 回环检测有可能导致机器人的位姿发生跳变.



**改进思路**

将纯定位模式与建图拆分开, 改成读取之前轨迹的地图进行匹配.

新的轨迹只进行位姿预测, 拿到预测后的位姿与之前轨迹的地图进行匹配, 新的轨迹不再进行地图的生成与保存. 同时将整个后端的功能去掉.

去掉了后端优化之后, 会导致没有重定位功能, 这时候可以将cartographer的回环检测(子图间约束的计算)部分单独拿出来, 做成一个重定位功能. 通过服务来调用这个重定位功能, 根据当前点云确定机器人在之前地图的位姿.

这样才是一个比较好的定位功能的思路.

#### 8.2.5 去ros的参考思路

有一些公司不用ros, 所以就要进行去ros的开发.

咱讲过数据是怎么通过cartographer_ros传到cartographer里去的, 只要仿照着cartographer_ros里的操作, 获取到传感器数据, 将数据转到tracking_frame坐标系下并进行格式转换, 再传入到cartographer里就行了.

cartographer_ros里使用ros的地方比较少, 只有在node.cc, sensor_bridge等几个类中进行使用, 只需要改这个类接受数据的方式以及将ros相关的格式修改一下就行了.








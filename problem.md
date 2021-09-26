
## 几种计算相对位姿的方式

### 节点在global坐标系下的位姿
```c++
const transform::Rigid3d optimized_pose(
    GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);

ComputeLocalToGlobalTransform() // 传入的始终是data_.global_submap_poses_2d 

// 只有在第一次优化完之后才有数据
data_.global_submap_poses_2d 

```
节点在global坐标系下的第一帧的位姿,就是这个节点在local坐标系下的位姿.

之后的节点在global坐标系下的位姿 是通过 global到local的坐标变换乘以local坐标系下的位姿得到的.

### submap在global坐标系下的位姿
```c++
optimization_problem_->AddSubmap(
    trajectory_id, transform::Project2D(
                        ComputeLocalToGlobalTransform(
                            data_.global_submap_poses_2d, trajectory_id) *
                        insertion_submaps[0]->local_pose()));

optimization_problem_->AddSubmap(
    trajectory_id,
    first_submap_pose *
        constraints::ComputeSubmapPose(*insertion_submaps[0]).inverse() *
        constraints::ComputeSubmapPose(*insertion_submaps[1]));
```
子图在global坐标系下的第一帧的位姿,就是这个子图在local坐标系下的位姿.

之后的子图在global坐标系下的位姿 是通过 global到local的坐标变换乘以 第一个子图到第二个子图在local坐标系下的位姿变换 得到的.


### 约束

#### 子图内约束
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

#### 子图间约束

先通过子图在global坐标系下的坐标的逆, 乘以节点在global坐标系下的坐标, 获取子图原点在glboal坐标系下指向节点的相对坐标变换.

然后根据子图在local坐标系下的位姿乘以这个坐标变换, 得到节点在local坐标系下的预测位姿, 在通过分枝定界粗匹配与ceres的精匹配, 对这个节点位姿进行校准, 校准后的位姿还是local坐标系下的.

最后, 通过子图在local坐标系下位姿的逆, 乘以这个节点校准后的位姿, 得到子图间约束, 是local坐标系下, **子图原点指向节点的相对坐标变换**.

```c++
const transform::Rigid2d initial_relative_pose =
    optimization_problem_->submap_data()
        .at(submap_id).global_pose.inverse() *
    optimization_problem_->node_data().at(node_id).global_pose_2d;

const transform::Rigid2d initial_pose =
    ComputeSubmapPose(*submap) * initial_relative_pose;

const transform::Rigid2d constraint_transform =
    ComputeSubmapPose(*submap).inverse() * pose_estimate;
```

### 4种计算相对位姿的方式总结
1. 节点 通过`GetLocalToGlobalTransform * constant_data->local_pose`进行global下位姿的计算
2. 子图 通过对前一个子图到后一个子图的坐标变换进行累计, 得到子图在global坐标系下的位姿
3. 子图内约束 local坐标系系下, 子图原点指向节点间的坐标变换
4. 子图间约束 根据global坐标计算初值, 然后通过分支定界算法粗匹配与ceres的精匹配, 获取校准后的位姿, 最后计算local坐标系系下, 子图原点指向校准后的节点间的坐标变换


## 残差项的计算
### 第一种残差 将节点与子图原点在global坐标系下的相对位姿 与 约束 的差值作为残差项

- 第一种坐标变换: 节点与子图原点在global坐标系下的坐标变换
- 第二种坐标变换: 子图内约束与子图间约束

这2个坐标变换的差形成残差.

### 第二种残差 landmark的 与 landmark数据插值出来的节点相对位姿 的差值作为残差项

- 第一种坐标变换: landmark数据的时间在2个节点位姿中插值出来的相对位姿
- 第二种坐标变换: landmark数据中的landmark_to_tracking_transform_

这2个坐标变换的差形成残差.

### 第三种残差 节点与节点间在global坐标系下的相对坐标变换 与 通过里程计数据插值出的相对坐标变换 的差值作为残差项

- 第一种坐标变换: 相邻2个节间在global坐标系下的坐标变换
- 第二种坐标变换: 再分别计算这2个节点的时间在里程计数据队列中插值得到的2个里程计位姿, 计算这2个里程计位姿间的坐标变换

这2个坐标变换的差形成残差.

### 第四种残差 节点与节点间在global坐标系下的相对坐标变换 与 相邻2个节点在local坐标系下的相对坐标变换 的差值作为残差项

- 第一种坐标变换: 相邻2个节间在global坐标系下的坐标变换
- 第二种坐标变换: 相邻2个节点在local坐标系下的坐标变换

这2个坐标变换的差形成残差.

### 第五种残差 节点与gps坐标系原点在global坐标系下的相对坐标变换 与 通过gps数据进行插值得到的相对坐标变换 的差值作为残差项

- 第一种坐标变换: 节点对应的时刻在gps数据中插值得到的gps相对于gps坐标系原点的位姿
- 第二种坐标变换: 节点在global坐标系下 与 gps坐标系原点 的相对坐标变换

这2个坐标变换的差形成残差.

## 优化后
### 优化前后的坐标变化
第一帧子图在global坐标系下的位姿不会被优化所改变

节点和之后的子图的原点在global坐标系下的位姿是会被优化改变的


```c++
problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
```

由于优化前后第一个子图原点在global坐标系下的位姿是不会发生变化的, 所以global坐标系是不会改变的, 改变的只有子图与节点在global坐标系下的位姿(除了第一帧子图位姿)


### ComputeLocalToGlobalTransform
ComputeLocalToGlobalTransform() 函数是计算最后一个子图原点的global坐标指向local坐标的坐标变换, 为什么要是最后一个子图的坐标呢?

local坐标系下的位姿都是相对与前一个节点的相对位姿, 时间越短, 相对位姿越准. 

优化后的最后一个子图位姿, 与当前节点的位姿在时间上距离最近. 

所以, 最后的一个优化后的子图位姿的相对坐标变换, 在时间上最接近当前节点与当前子图, 时间越短, 乘以相对坐标变换后得到的节点或者子图在global坐标系下的位姿越接近真实值.


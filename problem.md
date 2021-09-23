


## 约束
local坐标系下, 子图原点指向tracking_frame的坐标变换

## 节点在global坐标系下的位姿
```c++
const transform::Rigid3d optimized_pose(
    GetLocalToGlobalTransform(trajectory_id) * constant_data->local_pose);
```
## submap在global坐标系下的位姿
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

## 优化前后的坐标变化
第一帧子图在global坐标系下的位姿不会被优化所改变

节点和之后的子图的原点在global坐标系下的位姿是会被优化改变的

ComputeLocalToGlobalTransform() 函数是计算最后一个子图原点的global坐标指向local坐标的坐标变换, 而local坐标系是不会变的, 所以global坐标系是会发生改变的.

但是, 即使global坐标系变了, 第一个子图原点在global坐标系下的坐标还是不会发生变化的.

估计得画图...


```c++
problem.SetParameterBlockConstant(C_submaps.at(submap_id_data.id).data());
```
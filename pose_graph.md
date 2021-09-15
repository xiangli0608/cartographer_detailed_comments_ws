后端优化 笔记

## 1 什么是后端优化

### 什么是图
由节点和边组成的一种数据结构, 节点之间的关系可以是任意的, 图中任意两节点之间都可能相关(存在边).

### 什么是位姿图
SPA论文中对位姿图的定义: 位姿图是一组通过非线性约束连接的机器人位姿, 这些非线性约束是从对附近位姿共有的特征的观察中获得的.

位姿图是一种图, 节点代表位姿, 边代表2个位姿间的相对坐标变换(也叫约束).

### 什么是优化
由于前端里程计会有累计误差, 那有没有一种方法可以将这种累计误差减小甚至消除掉呢?

这就是优化的作用. 

SPA论文中的定义: 其目标是寻找一个节点间的配置关系使得节点间约束的测量概率最大.

通过图结构, 根据约束与所有的顶点, 将所有的误差减小, 并且平均分散到每个节点上去, 以达到减小整体误差的作用.

### 如何做优化呢

SPA优化

论文存放在 cartographer_detailed_comments_ws/src/cartographer/docs/2010 - Efficient Sparse Pose Adjustment for 2D Mapping - Konolige et al.pdf

论文翻译 [https://blog.csdn.net/u014527548/article/details/106238658](https://blog.csdn.net/u014527548/article/details/106238658)

残差方程


目标
第一: 确定2个节点在global坐标系下的相对位姿变换
第二: 通过其他方式再次获取这2个节点的相对位姿变换

求 这2个相对位姿变换的差 的最小二乘问题.

求解之后会得到一个增量 $\Delta{x}$ , 意为将当前位姿加上这个增量后会得到优化后的位姿

### cartographer代码里实现位姿图优化的思路
cartorgapher使用ceres进行位姿图优化, ceres求解的是残差的和. 
单一约束是不能进行优化的, 至少要有2个约束才能进行基于ceres的位姿图优化.

所以现在问题就是如何找到2个约束
第一个约束, 根据SPA论文, 就是节点global坐标系下的相对坐标变换.

所以, 第二个约束怎么求呢?

cartographer中设置了如下5种第二个约束及残差, 共同进行优化.

- 第一种残差 将节点与子图原点在global坐标系下的相对位姿 与 约束 的差值作为残差项
- 第二种残差 landmark数据与节点位姿间的相对坐标变换 与 landmark观测 的差值作为残差项
- 第三种残差 节点与节点间在global坐标系下的相对坐标变换 与 通过里程计数据插值出的相对坐标变换 的差值作为残差项
- 第四种残差 节点与节点间在global坐标系下的相对坐标变换 与 相邻2个节点在local坐标系下的相对坐标变换 的差值作为残差项
- 第五种残差 节点与gps坐标系原点在global坐标系下的相对坐标变换 与 通过gps数据进行插值得到的相对坐标变换 的差值作为残差项

###
所以, 整个后端优化问题, 可以简化成
- 求节点在global坐标系下的位姿 (节点)
- 求节点的约束(子图内约束与子图间约束, 子图间约束又称为回环约束) (边) 
- 求其他四种残差共同构建残差方差
- 如何进行优化 

节点在global坐标系下的位姿与 子图内约束
  PoseGraph2D::ComputeConstraintsForNode
子图间约束又称为回环约束
  ConstraintBuilder2D
  FastCorrelativeScanMatcher2D
构建残差方差与进行优化
  OptimizationProblem2D::Solve


### 后端整体函数流程
pose_graph_2d
  AddNode
    GetLocalToGlobalTransform
      ComputeLocalToGlobalTransform
    AppendNode
      AddTrajectoryIfNeeded
      CanAddWorkItemModifying
      添加节点
      保存地图指针

    AddWorkItem
      Task与ThreadPool
      work_queue_
    DrainWorkQueue
      ComputeConstraintsForNode
        InitializeGlobalSubmapPoses
        计算子图内约束
        计算子图间约束(回环检测)
          当前节点与所有已经完成的子图进行约束的计算
          计算所有节点与刚完成子图间的约束
          ComputeConstraint
            MaybeAddConstraint (并没有计算, 而是将计算放在线程池中了)
            MaybeAddGlobalConstraint (并没有计算, 而是将计算放在线程池中了)


  线程池中都有那些任务, 画个图
    DrainWorkQueue 1个线程
      ComputeConstraintsForNode
        InitializeGlobalSubmapPoses
        计算子图内约束
        计算子图间约束(回环检测)
          当前节点与所有已经完成的子图进行约束的计算
          计算所有节点与刚完成子图间的约束
          ComputeConstraint

    scan_matcher_task(初始化匹配器与多分辨率地图)
    constraint_task(进行约束的计算)
    finish_node_task_()
    when_done_task_(进行优化)

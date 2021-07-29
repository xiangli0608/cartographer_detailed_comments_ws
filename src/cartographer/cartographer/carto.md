
1. common
  - code:
    - 采样器的实现
    - 读取lua文件的实现
    - 线程任务处理相关功能的实现
    - 线程池的实现
    - 其他的数据类型
  - internal:
    - 阻塞队列的实现
    - ceres的options的设置
    - rate timer
  - proto: ceres匹配的设置的proto
2. io
  - internal:
  - code:
3. mapping
  - code: 
    - 整体建图的外部接口
    - 整体建图的接口类
    - 传感器数据处理功能的接口类
    - 后端的接口类(前端的接口不在这)
    - 地图的接口类
    - 将雷达数据写成地图的接口类
    - 2d与3d共用的地图存储值转换相关的功能实现
    - 2d与3d共用的位姿预测(pose_extrapolator)的接口类与功能实现
    - 前端后端共用的数据结构
  - 2d: 2d栅格地图相关的数据结构
  - 3d: 3d网格地图相关的数据结构
  - internal:
    - code: 
      - 连接前端与后端的功能的实现
      - 传感器数据处理功能的实现
      - 雷达数据的时间同步
      - motion filter
      - 轨迹连接状态的功能的实现
      - 一些共用的数据结构
    - 2d:
      - code:
        - 2d 前端的实现
        - 2d 前端结果的处理
        - 2d 后端的实现
        - tsdf格式地图的数据结构
        - 将雷达数据写成tsdf地图的实现
      - scan_matching:
        - 2d 残差方程的实现
        - 2d 相关性扫描匹配器的实现
        - 2d ceres扫描匹配器的实现
        - 2d 分支定界扫描匹配器的实现
    - 3d:
      - code:
        - 3d 前端的实现
        - 3d 前端结果的处理功能的实现
        - 3d 后端的实现
      - scan_matching:
        - 3d 残差方程的实现
        - 3d 相关性扫描匹配器的实现
        - 3d ceres扫描匹配器的实现
        - 3d 分支定界扫描匹配器的实现
        - 3d 旋转直方图匹配器的实现
    - scan_matching: 相关性扫描匹配器的接口类
    - constraints: 2d与3d构建约束的功能函数
    - optimization
      - code: 构建2d与3d优化问题的功能函数
      - cost_functions: 2d与3d的残差方程
    - testing: 测试某些功能
  - proto:
    - pose_graph: 位姿图的配置的proto格式的数据类型
    - scan_matching: ceres匹配器与分枝定界匹配器的配置的proto格式的数据类型
    - code: 地图相关的数据结构的proto格式的数据类型
4. sensor
 - code: 自定义的传感器相关的数据结构
 - internal: 处理传感器数据的相关功能函数
 - proto: 传感器相关的数据结构的proto格式的数据类型
5. transform
 - code: 坐标变换相关的数据类型,与坐标变换相关函数
 - proto: 坐标变换相关数据结构的proto格式的数据类型
6. ground_truth
7. metrics
8. cloud

先讲 map_builder.cc

照着sensor_bridge讲 CollatedTrajectoryBuilder 有个回调函数

再讲 sensor::Collator

OrderedMultiQueue

BlockingQueue

最后 GlobalTrajectoryBuilder

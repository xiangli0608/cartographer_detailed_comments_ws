
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

### 第2讲 node_main.cc文件讲解

#### Todo Tree说明

- ?: 目前我不理解的地方的标注
- note: 重点库, 重点概念, 重点功能的语句的标注
- c++11: c++11新标准与c++不常用语法的标注
- Step: 代码步骤说明

#### gflag简介
gflags是google开源的命令行标记处理库
命令行标记,顾名思义就是当运行一个可执行文件时, 由用户为其指定的标记, 形如：
`fgrep -l -f ./test ccc jjj`
`-l`与`-f`是**命令行标记**, 而`ccc`与`jjj`是命令行参数, 因为这两者不是以`-`开头的.

一般的一个可执行文件, 允许用户为其传入命令行标记以及参数.
如上述例子, `-l`是一个不带参数的标记, `-f`是一个带了参数`./test`的标记, 而gflags可以解析这些标记以及参数并将其存储在某些数据结构中.

gflags主要支持的参数类型包括bool, int32, int64, uint64, double, string等
定义参数通过DEFINE_type宏实现, 该宏的三个参数含义分别为命令行参数名, 参数默认值, 以及参数的帮助信息
当参数被定义后, 通过**FLAGS_name**就可访问到对应的参数

> 引用于 https://zhuanlan.zhihu.com/p/108477489

#### glog简介

Google glog是一个应用级别的日志系统库.它提供基于C++风格的流和各种辅助宏的日志API.支持以下功能：
- 参数设置, 以命令行参数的方式设置标志参数来控制日志记录行为
- 严重性分级, 根据日志严重性分级记录日志 - INFO WARNING ERROR FATAL
- 可有条件地记录日志信息 - LOG_IF LOG_EVERY_N LOG_IF_EVERY_N LOG_FIRST_N
- 条件中止程序。丰富的条件判定宏, 可预设程序终止条件 - CHECK宏
- 异常信号处理。程序异常情况, 可自定义异常处理过程
- 支持debug功能 
- 自定义日志信息
- 线程安全日志记录方式
- 系统级日志记录
- google perror风格日志信息
- 精简日志字符串信息

最终的结果不仅会在屏幕终端显示出来, 同时会将log日志写入到/tmp/<program name>.<hostname>.<user name>.log.<severity level>.<date>.<time>.<pid>这个文件中

glog比较语句的源码

```c++
  #define CHECK_EQ(val1, val2) CHECK_OP(_EQ, ==, val1, val2)
  #define CHECK_NE(val1, val2) CHECK_OP(_NE, !=, val1, val2)
  #define CHECK_LE(val1, val2) CHECK_OP(_LE, <=, val1, val2)
  #define CHECK_LT(val1, val2) CHECK_OP(_LT, < , val1, val2)
  #define CHECK_GE(val1, val2) CHECK_OP(_GE, >=, val1, val2)
  #define CHECK_GT(val1, val2) CHECK_OP(_GT, > , val1, val2)
```

# cartographer 超详细注释代码

## 项目简介
基于2021.04.20日在github上下载的master的代码的注释版本
1. 在代码中进行了规范, 超详细的注释, 帮助大家更好的理解cartographer
  **注释多数在 .cc 文件中, .h文件中存在cartographer原本的注释**
2. 对cartographer中的关键概念进行了解释与说明
3. 对cartographer中使用的**c++11新标准**语法进行了标注与说明
4. 对3d的cartographer添加了实时显示三维点云地图与保存地图的功能

## 实时显示三维点云地图与保存地图的功能简述
显示三维点云地图需要是在建3d轨迹的情况下才可以显示, 点云地图默认发布在 `/point_cloud_map` 话题中.

默认关闭了保存pcd格式点云的功能, 如需要这个功能, 需要将node.cc文件的第1029行的 `constexpr bool save_pcd = false;` 改成true.

编译之后在建图结束后通过调用 `/write_state` 服务, 会在生成pbstream文件的同时也对pcd文件进行保存, 保存目录与pbstream文件同一个文件夹.

个人水平有限, 如果改完的代码有问题请您联系我进行改正.

![pcd_map](src/cartographer/docs/pcd_map.png)

## 课程简介与购买链接

如果是购买了本项目的对应**课程**的同学, 还会有
- 逐行讲解cartographer代码的学习视频
- 帮助理解代码结构的的思维导图
- 难理解部分的代码通过画图来对原理进行讲解
- 关键部分的公式推导课件

### 课程简介
激光SLAM是机器人、自动驾驶领域的底层核心技术, 招聘规模越来越大, 薪资也一路水涨船高, 应计算机视觉life公众号粉丝要求, 我们经过几个月打磨, 推出了激光SLAM独家逐行源码解析课程《Cartographer从入门到精通: 原理深剖+源码逐行详解》

### 课程大纲

#### 第一章 编译运行及调参指导
1. Cartographer论文带读
2. 代码的编译与运行
3. 配置文件参数详解与调试
4. 代码基础介绍
  a. Cartographer中使用的C++11新标准的语法
  b. 关键概念与关键数据结构

#### 第二章 Cartographer_ros代码阅读
1. 入口函数分析
  a. gflags简介
  b. glog简介
  c. 自定义log的格式
  d. 配置文件的加载
2. ROS接口
  a. SLAM的启动, 停止, 数据的保存与加载
  b. 话题的订阅与发布
  c. 传感器数据的走向与处理
  d. 服务的处理
  e. 可视化信息的设置与发布
  f. ROS中的数据类型与cartorgrapher中的数据类型的转换

#### 第三章 传感器数据的处理过程分析
1. 传感器数据的传递过程分析
  a. Cartographer_ros中的传感器数据的传递过程分析
  b. Cartographer中的传感器数据的传递过程分析
2. 2D情况下的激光雷达数据的预处理
  a. 多个激光雷达数据的时间同步与融合
  b. 激光雷达数据运动畸变的校正与无效点的处理
  c. 将点云据根据重力的方向做旋转变换
  d. 对变换后的点云进行z方向的过滤
  e. 对过滤后的点云做体素滤波
3. 3D情况下的激光雷达数据的预处理
  a. 多个激光雷达数据的时间同步与融合
  b. 对融合后的点云进行第一次体素滤波
  c. 激光雷达数据运动畸变的校正与无效点的处理
  d. 分别对点云的hit点与miss点进行第二次体素滤波
  e. 将点云根据预测出来的先验位姿做旋转平移变换

#### 第四章 2D扫描匹配
1. 扫描匹配理论过程分析
2. 生成2D局部地图
  a. 查找表的实现
  b. 概率地图的实现与地图坐标系的定义
  c. 概率地图的更新方式
  d. 如何将点云插入到概率地图中
3. 基于局部地图的扫描匹配
  a. 基于IMU与里程计的先验位姿估计
  b. 根据机器人当前姿态将预测出来的6维先验位姿投影成水平面下的3维位姿
  c. 对之前预处理后的点云进行自适应体素滤波
  d. 使用实时的相关性扫描匹配进行粗匹配
  e. Ceres的简介与编程练习
  f. 进行基于图优化的扫描匹配实现精匹配
4. 扫描匹配的结果的处理
  a. 将匹配的结果与生成的局部地图加入到后端的位姿图结构中
  b. 调用传入的回调函数进行扫描匹配结果与局部地图的保存

#### 第五章 2D后端优化
1. 后端优化的概念与理论过程分析
2. 向位姿图中添加顶点
3. 任务队列与线程池
4. 为顶点进行子图内与子图间约束的计算
5. 2D情况下的回环检测
  a. 使用滑动窗口算法生成指定分辨率的栅格地图
  b. 多分辨率栅格地图的生成
  c. 分支定界算法简介
  d. 基于多分辨率地图的分支定界粗匹配
6. 2D情况下的优化问题的构建与求解
  a. 几种计算相对位姿的方式总结
  b. 优化问题残差的构建思路
  c. 5种残差项的构建
  d. 使用Ceres进行全局优化
7. 优化后的分析

#### 第六章 代码总结与3D建图
1. 代码的全面总结
2. Cartographer的优缺点分析
3. TSDF地图
4. 3D网格地图
5. 3D扫描匹配
6. 3D后端优化

#### 第七章 地图保存与纯定位
1. Submap与ROS格式地图间的格式转换
2. ROS地图的发布
3. 纯定位模式

#### 第八章 调参总结与工程化建议
1. 调参总结
  a. 降低延迟与减小计算量
  b. 降低内存
  c. 常调的参数
2. 工程化建议
  a. 工程化的目的
  b. 提升建图质量
  c. 降低计算量与内存
  d. 纯定位的改进建议
  e. 去ros的参考思路

### 课程购买链接
扫描大纲图片上的二维码即可查看课程购买地址, 图片刷不出来也可以通过下面这个链接进行更详细的了解与购买

[购买链接](https://mp.weixin.qq.com/s/5RV3ZWFiFykIBjSllFQcAQ)

## 推荐编译环境
- ubuntu 16.04/18.04 版本
- ROS Kinetic/Melodic 版本
- vscode

vscode中推荐的插件有: 
- C/C++
- C++ Intellisense
- Doxygen Documentation Generator
- Msg Language Support
- XML Tools
- Todo Tree: 注释的高亮显示


## 运行相关命令

### 2d建图指令
`roslaunch cartographer_ros lx_rs16_2d_outdoor.launch`

### 保存2d轨迹,并生成ros格式的地图
`./finish_slam_2d.sh`

### 纯定位模式
`roslaunch cartographer_ros lx_rs16_2d_outdoor_localization.launch`

### 3d建图指令
`roslaunch cartographer_ros lx_rs16_3d.launch`

### 保存3d轨迹
`./finish_slam_3d.sh`

### 使用asset生成ros格式的2d栅格地图
`roslaunch cartographer_ros assets_writer_2d.launch`

### 使用asset生成3d点云地图
`roslaunch cartographer_ros assets_writer_3d.launch`

### landmark使用示例
`roslaunch cartographer_ros landmark_mir_100.launch`


**by lixiang**

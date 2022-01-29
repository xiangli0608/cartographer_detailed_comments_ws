

# cartographer从入门到精通: 原理深剖+源码逐行讲解

## 第一章 论文带读、编译运行及调参指导

### 1.1【开课直播】Cartographer论文带读 

### 1.2 代码的编译与运行

#### 官方文档

官方文档的地址为

- https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html
- https://google-cartographer.readthedocs.io/en/latest/index.html

#### 系统环境

- 推荐在ubuntu16.04或者18.04上进行编译

- 推荐使用刚装好的ubuntu系统

- 将ubuntu的软件源设置成清华的或者其他的中国境内的源, 更换完源之后要执行一下 `sudo apt-get update`

- 照着wiki.ros.org安装完对应版本的**ROS**

- 并在.bashrc中的末尾添加这条句语句, 要把`<distro>`改成对应的ROS版本 
  `source /opt/ros/<distro>/setup.bash`
  
- **其中 `sudo rosdep init; rosdep update`这条语句执行不成功, 不管他, 不是必须执行的**





链接：https://pan.baidu.com/s/1q9f5aNTfrbpFpwGCwD6E4A 
提取码：slam





#### cartographer依赖库的安装

cartographer的依赖库脚本完成一键安装.

如下是具体步骤: 

第一步: 下载安装包
在网盘链接的里 数据集与安装包 文件夹 中下载 cartographer_install-20210613.zip, 放到ubuntu系统里并解压

第二步: 打开终端
进入到cartographer_install文件夹, 在文件夹内部空白处单击鼠标右键, 打开终端

第三步 使用脚本完成依赖项的安装
在终端中输入 `./auto-carto-build.sh`

如果提示脚本不是一个可执行的文件, 那就通过 `chmod +x auto-carto-build.sh` 为这个脚本添加可执行权限即可, 之后在通过`./auto-carto-build.sh`执行脚本.

解释说明

脚本里进行了4个库的编译与安装
cartographer的依赖库有3个,  分别是abseil-cpp, ceres-solver, protobuf.

在脚本的最后, 也将cartographer安装到系统里了. 安装cartographer后, 就可以在工作空间中只对cartographer_ros进行编译了, 这样修改launch和lua文件后就不需要执行一次编译指令了.

**注意: 如果之前安装过 ceres-solver, protobuf, cartographer 的话, 执行脚本时可能会报有些函数没有定义的错误, 这是由于protobuf或者cartographer不一致版本导致的, 可以先将之前安装的库卸载掉, 再执行安装依赖的脚本**

一定要确保脚本执行之后没有错误输出, 否则之后编译代码会编译不通过.

#### cartorgapher的编译

第一步: 下载注释的代码 

讲师注释的代码的github地址为 
https://github.com/xiangli0608/cartographer_detailed_comments_ws

在某个文件夹内部空白处单击鼠标右键, 打开终端, 输入如下执行进行代码的下载.
`git clone https://github.com/xiangli0608/cartographer_detailed_comments_ws.git`

**注意**, 一定要通过 git clone 的方式下载代码, 因为代码的注释是处于更新状态的, 要定期通过 `git pull origin master` 来更新代码.

要是在github中下载代码的zip压缩文件, 这样下载的代码里不含有.git文件夹, 就不能通过 `git pull` 进行代码的更新了.

如果执行 `git clone` 后没有下载代码, 是由于没有连上github导致的, 可以按 `ctrl c` 中断当前下载, 再次执行 `git clone` , 重复几次试试, 如果还不行, 就在群里问问吧.

第二步: cartorgapher的编译

进入到cartographer_detailed_comments_ws内, 在文件夹内打开终端, 输入
`./catkin_make.sh`
完成对cartographer与cartographer_ros的编译.
注意: 一定要确保之前的依赖项全部安装成功, 否则这里的编译就会报错.

#### cartographer的运行
下载数据集

本次使用的bag有2个, 



已经上传到qq群文件中, 名字为 bag-2021-06-13.zip , 请提前下载好并放到ubuntu中.

- rslidar-outdoor-gps.bag 这个是讲师自己录制的数据, 包括了速腾16线点云数据, 由点云数据生成的单线雷达数据, imu, 里程计, rtk/gps, tf 与 tf_static.
- landmarks_demo_uncalibrated.bag 是从cartographer官网上下载的数据, 旨在展示一下如何在cartographer中使用landmark.

为了防止群文件的内容过期, 也将这两个bag的百度云链接放在下面, 如果有需要可以自行下载. 下载的文件名字可能会有出入, 重命名一下就好.

**rslidar-outdoor-gps.zip**

链接：https://pan.baidu.com/s/1eOywWPnpIFaly9eawZ8Nlw  提取码：slam 

**landmarks_demo_uncalibrated.zip**

链接：https://pan.baidu.com/s/1JGIh6u8BMeGKvUD5KthZMQ  提取码：slam 

#### 1.3.2 了解bag文件

播放bag文件需要在**bag的文件夹内**启动三个终端

- 第一个终端执行`roscore`
- 第二个终端执行 `rosbag play rslidar-outdoor-gps.bag`
- 第三个终端执行 `rqt`

注意, rqt的所有显示插件都在菜单栏的plugins内, 再点击 visualization, 再点击 tf tree, 即可显示视频中的 tf树的可视化界面.

在第三个终端按 `ctrl c`, 结束rqt的显示, 再启动 `rviz`

rviz左侧最顶上的 Fixed frame 除了下拉菜单可以选择之外, 还是可以手动输入的, 现在手动输入成 odom , 

rviz左侧下方的 Add 按钮, 可以添加显示的插件, 自己选择添加 pointcloud2, laserscan等格式的数据, 之后把插件订阅的topic选择一下, 如果不会就自己摸索一下, 很简单的.

#### 1.3.3 重新录制bag

由于rslidar-outdoor-gps.bag这个bag的tf是从odom开始的, 不好做之后修改cartographer发布坐标系的演示, 所以这里重新录制一个tf不带odom->footprint的新的bag.

通过视频来进行操作即可.

#### 1.3.4 运行cartographer

##### 2d建图

**修改bag文件的地址**

第一步: 把bag的默认地址手动修改成自己存放bag的目录, 这个由于每个人的用户名, 文件夹名都不一样, 所以需要每个人对自己的电脑做相应的修改.

例如我的bag文件放在 `/home/lx/bagfiles`文件夹内, bag的名字为rslidar-outdoor-gps-notf, 所以bag的地址设置为`/home/lx/bagfiles/rslidar-outdoor-gps-notf.bag`.

如果不能确定bag所在的地址, 在bag文件夹内部打开终端, 输入`pwd` 会显示出当前文件夹的地址.

进入到cartographer_detailed_comments_ws文件夹内, 在文件夹内右键打开终端, 输入

```
./catkin_make.sh
source install_isolated/setup.bash
rospack profile
roslaunch cartographer_ros lx_rs16_2d_outdoor.launch
```

由于修改了launch文件, 所以需要重新编译一下.

**source命令的目的**
source命令是设置当前的终端 可以执行的包与节点的地址集合的, 不执行 `source install_isolated/setup.bash` 这个命令, 直接启动`roslaunch` 时就会出现 找不到包或找不到节点的错误提示.

**rospack profile 命令的目的**
新下载的包在编译之后, 有时没有被加载到ros的包的地址集合中,.
这时即使执行力`source`指令, 在启动` roslaunch` 时也可能会报错, 提示找不到包或者可执行节点.
通过 `rospack profile` 命令可以将新下载的包加载到ros的包的地址集合, 再执行 `roslaunch` 时就不会再报错了.

为了防止每次启动终端都要执行一次 `source install_isolated/setup.bash` 这个指令, 我们可以将这个指令写到 ~/.bashrc 文件中, 每次启动终端时, 会自动source ~/.bashrc 这个文件, 这个文件中的所有内容都会被 source 一遍.

~ 代表用户文件夹, 例如我自己电脑的用户名为lx, 我自己的用户文件夹就是 /home/lx/ , 这个文件夹地址也可以用 ~ 来代替.

按 `ctrl h`可以显示隐藏文件, 找到 .bashrc 文件进行编辑, 在文件底部输入 

`source ~/path-to-carto/cartographer_detailed_comments_ws/install_isolated/setup.bash`

其中 `path-to-carto` 要改成自己实际的文件夹名字.

**保存2d轨迹,并生成ros格式的地图**

`./finish_slam_2d.sh`

其中map_dir是保存地图的文件夹名字, 可以改成自己想要保存的文件夹地址.

**纯定位模式**

`roslaunch cartographer_ros lx_rs16_2d_outdoor_localization.launch`

**3d建图指令**

`roslaunch cartographer_ros lx_rs16_3d.launch`

**保存3d轨迹**

`./finish_slam_3d.sh`

**landmark使用示例**

`roslaunch cartographer_ros landmark_mir_100.launch`

**使用asset生成ros格式的2d栅格地图**

`roslaunch cartographer_ros assets_writer_2d.launch`

**使用asset生成3d点云地图**

`roslaunch cartographer_ros assets_writer_3d.launch`





### 1.3 如何配置Launch与Lua文件

### 1.4 常见运行错误汇总与解决

#### launch 订阅话题的名字对不上时会提示这样的错误信息

launch文件中topic名字没设置对, 会在运行时产生 `Queue waiting for data ....` 这种问题.

```
[ INFO] [1625724722.715674522]: I0708 14:12:02.000000 20138 map_builder_bridge.cc:153] Added trajectory with ID '0'.
[ WARN] [1625724725.726943013]: W0708 14:12:05.000000 20138 node.cc:1267] Expected topic "points2" (trajectory 0) (resolved topic "/points2") but no publisher is currently active.
[ WARN] [1625724725.727046746]: W0708 14:12:05.000000 20138 node.cc:1267] Expected topic "imu" (trajectory 0) (resolved topic "/imu") but no publisher is currently active.
[ WARN] [1625724725.727145643]: W0708 14:12:05.000000 20138 node.cc:1280] Currently available topics are: /constraint_list,/submap_list,/scan_matched_points2,/rosout,/tf,/clock,/rosout_agg,/map,/trajectory_node_list,/landmark_poses_list,
[ WARN] [1625724739.386545399, 1606808654.481099623]: W0708 14:12:19.000000 20138 ordered_multi_queue.cc:232] Queue waiting for data: (0, points2)
[ WARN] [1625724739.994835822, 1606808655.086546848]: W0708 14:12:19.000000 20138 ordered_multi_queue.cc:232] Queue waiting for data: (0, points2)
[ WARN] [1625724740.588885518, 1606808655.682104741]: W0708 14:12:20.000000 20138 ordered_multi_queue.cc:232] Queue waiting for data: (0, points2)
[ WARN] [1625724741.195557158, 1606808656.288400347]: W0708 14:12:21.000000 20138 ordered_multi_queue.cc:232] Queue waiting for data: (0, points2)
```

错误信息截图展示如下

![error_log](C:/Users/tianc/Desktop/录课/课件/课件markdown/2-1-1-error_log.png) 

#### lua
lua文件中坐标系设置的不对, 导致tf树连不起来, 会在运行时产生 `passed to lookupTransform argument source_frame dose not exit` 这种问题.



### 1.5 参数的详解与调参总结

## 第二章 Cartographer_ros代码入门

### 看视频的注意事项

- **看视频不等于看懂代码:** 看视频只能大致听懂代码, 不是自己真正理解代码, 光看视频很容易看完就忘
- **要有自己的思考过程: ** 自己读代码 / 自己手动注释代码 / 写技术博客或者笔记
- 尽量不要跳着看视频, 有时跳着看会错过重要知识点, 可以1.25/2.0**倍速观看**
- 可以只听视频, 自己看代码
- 每次听代码讲解的视频之前, 要先将**代码更新**一下

### 2.1 VScode的配置与使用

### 2.2 Cartographer_ros的功能与框架分析

### 2.3 CMakeLists.txt文件讲解

听不懂没关系, 不影响cartographer的学习

### 2.4 Main函数使用的依赖库简介

#### Todo Tree说明

- **?**: 目前我不理解的地方的标注
- **note**: 重点库, 重点概念, 重点功能的语句的标注
- **c++11**: c++11新标准与c++不常用语法的标注
- **Step**: 代码步骤说明

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


### 2.5 Main函数讲解

#### 函数指针

```c++
int function(int a, int b)
{
  // 执行代码
}

int main(void)
{
  int (*FP)(int, int); // 函数指针的声明方式
  FP = function;       // 第一种赋值方法
  // FP = &function;   // 第二种赋值方法
  FP(1,2);             // 第一种调用方法
  // (*FP)(1,2);       // 第二种调用方法
  return 0;
}

class MyClass {
public:
    int Fun(int a, int b) {
        cout << "call Fun" << endl;
        return a + b;
    }
};
  
int main() {
    MyClass* obj = new MyClass;
    int (MyClass::*pFun)(int, int) = &MyClass::Fun;    // 成员函数指针的声明与初始化
    (obj->*pFun)(1, 2);						    // 通过对象来调用成员函数指针
    delete obj;
    return 0;
}
```

#### lambda表达式 

```c++
[函数对象参数 = & ] (操作符重载函数参数) mutable 或 exception 声明 (-> 返回值类型) {函数体}

eg1
[node, handler, trajectory_id, topic](const typename MessageType::ConstPtr& msg)
{
  (node->*handler)(trajectory_id, topic, msg);
}

eg2
const Rigid3d tracking_to_local = [&] {
  // 是否将变换投影到平面上
  if (trajectory_data.trajectory_options.publish_frame_projected_to_2d) {
    return carto::transform::Embed3D(
        carto::transform::Project2D(tracking_to_local_3d));
  }
  return tracking_to_local_3d;
}();
```

### 2.6 配置文件的加载


## 第三章 Cartographer_ros代码进阶

### 3.1 开始轨迹的相关处理


#### 3.1.1 node_main.cc

```c++
auto map_builder = cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);

Node node(node_options, std::move(map_builder), &tf_buffer, FLAGS_collect_metrics);
```

#### 3.1.2 Node类的构造函数

```c++
Node::Node(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer, const bool collect_metrics)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, std::move(map_builder), tf_buffer)
{
  // Step: 1 声明需要发布的topic
  // Step: 2 声明发布对应名字的ROS服务, 并将服务的发布器放入到vector容器中
  // Step: 3 处理之后的点云的发布器
  // Step: 4 进行定时器与函数的绑定, 定时发布数据
}
```

#### 3.1.3 MapBuilderBridge类的构造函数

```c++
MapBuilderBridge::MapBuilderBridge(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer)
    : node_options_(node_options),
      map_builder_(std::move(map_builder)),
      tf_buffer_(tf_buffer) {}
```

#### 3.1.4 MapBuilder类的构造函数

```c++
MapBuilder::MapBuilder(const proto::MapBuilderOptions& options)
    : options_(options), thread_pool_(options.num_background_threads()) {
        
  // 2d位姿图(后端)的初始化
  if (options.use_trajectory_builder_2d()) {
    pose_graph_ = absl::make_unique<PoseGraph2D>(
        options_.pose_graph_options(),
        absl::make_unique<optimization::OptimizationProblem2D>(
            options_.pose_graph_options().optimization_problem_options()),
        &thread_pool_);
  }

  if (options.collate_by_trajectory()) {
    sensor_collator_ = absl::make_unique<sensor::TrajectoryCollator>();
  } else {
    sensor_collator_ = absl::make_unique<sensor::Collator>();
  }
}
```


### 3.2 数据采样器

### 3.3 订阅话题与注册回调函数

### 3.4 传感器数据的走向与传递

## 第四章 传感器的格式转换与类型变换

### 4.1 三维刚体坐标变换的理论与实现


<img src="2-2-1-三维刚体1.jpg" alt="2-2-1-三维刚体1" style="zoom: 33%;" />

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/2-2-2-三维刚体2.jpg" alt="2-2-2-三维刚体2" style="zoom: 33%;" />


### 4.2 IMU数据、里程计数据与Landmark数据的格式转换与坐标变换

### 4.3 GPS数据的处理与编程实践


#### ECEF坐标系

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/2-2-3-ecef.png" alt="2-2-3-ecef.png" style="zoom: 50%;" />

### 4.4 雷达数据的格式转换与坐标变换


#### LaserScan与MultiEchoLaserScan的数据类型

```
$ rosmsg show sensor_msgs/LaserScan 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities

$ rosmsg show sensor_msgs/MultiEchoLaserScan 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
sensor_msgs/LaserEcho[] ranges
  float32[] echoes
sensor_msgs/LaserEcho[] intensities
  float32[] echoes
```


### 4.5 Cartographer_ros中的传感器数据的传递过程总结

## 第五章 传感器数据的分发处理

### 5.1 Cartographer中的传感器数据分发过程分析

### 5.2 数据分发器相关类的构造与初始化


#### 3.2.1 node_main.cc的StartTrajectoryWithDefaultTopics函数

```c++
node.StartTrajectoryWithDefaultTopics(trajectory_options);
```

#### 3.2.2 Node类的StartTrajectoryWithDefaultTopics函数

```c++
StartTrajectoryWithDefaultTopics();
  AddTrajectory(options);
    const int trajectory_id = map_builder_bridge_.AddTrajectory(expected_sensor_ids, options);
    LaunchSubscribers(options, trajectory_id); // 开始订阅传感器数据话题
```

#### 3.2.3 MapBuilderBridge类的AddTrajectory函数

```c++
int MapBuilderBridge::AddTrajectory() {
  // Step: 1 开始一条新的轨迹, 返回新轨迹的id,需要传入一个函数
  const int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options.trajectory_builder_options,
      [this]() {
        OnLocalSlamResult(trajectory_id, time, local_pose, range_data_in_local);
      });

  // Step: 2 为这个新轨迹 添加一个SensorBridge
  sensor_bridges_[trajectory_id] = absl::make_unique<SensorBridge>(
      trajectory_options.num_subdivisions_per_laser_scan,
      trajectory_options.tracking_frame,
      node_options_.lookup_transform_timeout_sec, 
      tf_buffer_,
      map_builder_->GetTrajectoryBuilder(trajectory_id)); // CollatedTrajectoryBuilder

  return trajectory_id;
}
```

#### 3.2.4 MapBuilder类的AddTrajectoryBuilder函数

```c++
int MapBuilder::AddTrajectoryBuilder(
    const std::set<SensorId>& expected_sensor_ids,
    const proto::TrajectoryBuilderOptions& trajectory_options,
    LocalSlamResultCallback local_slam_result_callback) {

    // CollatedTrajectoryBuilder初始化
    trajectory_builders_.push_back(absl::make_unique<CollatedTrajectoryBuilder>(
        trajectory_options, 
        sensor_collator_.get(), // sensor::Collator
        trajectory_id,
        expected_sensor_ids,
        CreateGlobalTrajectoryBuilder2D(
            std::move(local_trajectory_builder), trajectory_id,
            static_cast<PoseGraph2D*>(pose_graph_.get()),
            local_slam_result_callback, pose_graph_odometry_motion_filter)));

  return trajectory_id;
}

// 返回指向CollatedTrajectoryBuilder的指针
mapping::TrajectoryBuilderInterface *GetTrajectoryBuilder(
    int trajectory_id) const override {
    return trajectory_builders_.at(trajectory_id).get(); 
}
```

#### 3.2.5 SensorBridge类的构造函数

```c++
SensorBridge::SensorBridge(
    const int num_subdivisions_per_laser_scan,
    const std::string& tracking_frame,
    const double lookup_transform_timeout_sec, 
    tf2_ros::Buffer* const tf_buffer,
    carto::mapping::TrajectoryBuilderInterface* const trajectory_builder) // CollatedTrajectoryBuilder
    : num_subdivisions_per_laser_scan_(num_subdivisions_per_laser_scan),
      tf_bridge_(tracking_frame, lookup_transform_timeout_sec, tf_buffer),
      trajectory_builder_(trajectory_builder) // CollatedTrajectoryBuilder
      {} 
```

#### 3.2.6 CollatedTrajectoryBuilder类的构造函数

```c++
CollatedTrajectoryBuilder::CollatedTrajectoryBuilder(
    const proto::TrajectoryBuilderOptions& trajectory_options,
    sensor::CollatorInterface* const sensor_collator,   // sensor::Collator
    const int trajectory_id,
    const std::set<SensorId>& expected_sensor_ids,
    std::unique_ptr<TrajectoryBuilderInterface> wrapped_trajectory_builder) // GlobalTrajectoryBuilder
    : sensor_collator_(sensor_collator),		 		// sensor::Collator
      trajectory_id_(trajectory_id),
      wrapped_trajectory_builder_(std::move(wrapped_trajectory_builder)),  // GlobalTrajectoryBuilder
      last_logging_time_(std::chrono::steady_clock::now()) {
  
  // sensor::Collator的初始化
  sensor_collator_->AddTrajectory(
      trajectory_id, expected_sensor_id_strings,
      [this](const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
        HandleCollatedSensorData(sensor_id, std::move(data));
      });
}
```

在Collator构造的时候传入了一个函数 HandleCollatedSensorData()

#### 3.2.7 Collator类的AddTrajectory函数

```c++
void Collator::AddTrajectory(
    const int trajectory_id,
    const absl::flat_hash_set<std::string>& expected_sensor_ids,
    const Callback& callback) {
  for (const auto& sensor_id : expected_sensor_ids) {
    const auto queue_key = QueueKey{trajectory_id, sensor_id};
    queue_.AddQueue(queue_key,
                    // void(std::unique_ptr<Data> data) 带了个默认参数sensor_id
                    [callback, sensor_id](std::unique_ptr<Data> data) {
                      callback(sensor_id, std::move(data));
                    });
    queue_keys_[trajectory_id].push_back(queue_key);
  }
}

// note: CollatorInterface::Callback 2个参数
using Callback = std::function<void(const std::string&, std::unique_ptr<Data>)>;
```

在这将传入的Callback函数放入 queue_ 里, 并传入一个参数.


#### 3.2.8 OrderedMultiQueue类的AddQueue函数

```c++
void OrderedMultiQueue::AddQueue(const QueueKey& queue_key, Callback callback) {
  CHECK_EQ(queues_.count(queue_key), 0);
  queues_[queue_key].callback = std::move(callback);
}

// note: OrderedMultiQueue::Callback 1个参数
using Callback = std::function<void(std::unique_ptr<Data>)>;
```

这里的Callback是一个参数的.




### 5.3 顺序多队列与数据的分发处理


### 3.3 传感器数据走向分析

#### 3.3.1 Node类的HandleLaserScanMessage函数

```c++
void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::LaserScan::ConstPtr& msg) {
  map_builder_bridge_.sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}
```

#### 3.3.2 SensorBridge类的HandleLaserScanMessage函数

```c++
void SensorBridge::HandleLaserScanMessage(
    const std::string& sensor_id, const sensor_msgs::LaserScan::ConstPtr& msg) {
  carto::sensor::PointCloudWithIntensities point_cloud;
  carto::common::Time time;
  std::tie(point_cloud, time) = ToPointCloudWithIntensities(*msg);
  HandleLaserScan(sensor_id, time, msg->header.frame_id, point_cloud);
}

void SensorBridge::HandleRangefinder(
    const std::string& sensor_id, const carto::common::Time time,
    const std::string& frame_id, const carto::sensor::TimedPointCloud& ranges) {
  if (sensor_to_tracking != nullptr) {
    trajectory_builder_->AddSensorData(
        sensor_id, carto::sensor::TimedPointCloudData{
                       time, 
                       sensor_to_tracking->translation().cast<float>(),
                       carto::sensor::TransformTimedPointCloud(
                          ranges, sensor_to_tracking->cast<float>())} ); // 强度始终为空
  }
}
```

**SensorBridge的trajectory_builder_是指向CollatedTrajectoryBuilder的指针**

#### 3.3.3 CollatedTrajectoryBuilder类的AddSensorData函数

```c++
void AddSensorData(
    const std::string& sensor_id,
    const sensor::TimedPointCloudData& timed_point_cloud_data) override {
    AddData(sensor::MakeDispatchable(sensor_id, timed_point_cloud_data));
}

void CollatedTrajectoryBuilder::AddData(std::unique_ptr<sensor::Data> data) {
  sensor_collator_->AddSensorData(trajectory_id_, std::move(data));
}
```

**CollatedTrajectoryBuilder的sensor_collator_是指向Collator的指针**

#### 3.3.4 Collator类的AddSensorData函数

```c++
void Collator::AddSensorData(const int trajectory_id,
                             std::unique_ptr<Data> data) {
  QueueKey queue_key{trajectory_id, data->GetSensorId()};
  queue_.Add(std::move(queue_key), std::move(data));
}
```

#### 3.3.5 OrderedMultiQueue类的Add函数 - 生成者

```c++
void OrderedMultiQueue::Add(const QueueKey& queue_key,
                            std::unique_ptr<Data> data) {
  auto it = queues_.find(queue_key);
  
  // 向数据队列中添加数据
  it->second.queue.Push(std::move(data));

  // 传感器数据的分发处理
  Dispatch();
}
```

#### 3.3.6 BlockingQueue类的Push函数 - 缓冲区(阻塞队列)

```c++
  void Push(T t) {
    // 将数据加入队列, 移动而非拷贝
    deque_.push_back(std::move(t));
  }
```

#### 3.3.7 OrderedMultiQueue类的Dispatch函数 - 消费者

```c++
void OrderedMultiQueue::Dispatch() {
  while (true) {
    const Data* next_data = nullptr;
    Queue* next_queue = nullptr;
    QueueKey next_queue_key;

    // 遍历所有的数据队列, 找到所有数据队列的第一个数据中时间最老的一个数据
    for (auto it = queues_.begin(); it != queues_.end();) {
      const auto* data = it->second.queue.Peek<Data>();
    } // end for

    // 正常情况, 数据时间都超过common_start_time
    if (next_data->GetTime() >= common_start_time) {
      last_dispatched_time_ = next_data->GetTime();
      // 将数据传入 callback() 函数进行处理,并将这个数据从数据队列中删除
      next_queue->callback(next_queue->queue.Pop());
    } 
  }
}
```

#### 3.3.8 CollatedTrajectoryBuilder类的HandleCollatedSensorData函数 - 消费者

```c++
void CollatedTrajectoryBuilder::HandleCollatedSensorData(
    const std::string& sensor_id, std::unique_ptr<sensor::Data> data) {
  // 将排序好的数据送入 GlobalTrajectoryBuilder中的AddSensorData()函数中进行使用
  data->AddToTrajectoryBuilder(wrapped_trajectory_builder_.get());
}

void Dispatchable::AddToTrajectoryBuilder(
    mapping::TrajectoryBuilderInterface *const trajectory_builder) override {
    trajectory_builder->AddSensorData(sensor_id_, data_);
}
```

这里的wrapped_trajectory_builder_是指向GlobalTrajectoryBuilder2D类的指针.

从GlobalTrajectoryBuilder2D开始, 数据才真正走到SLAM的前端与后端部分.




### 5.4 阻塞队列与生产者消费者模式

### 5.5 传感器数据的分发处理过程总结

## 第六章 点云数据的预处理

### 6.1 传感器数据的走向

### 6.2. 2D情况下激光雷达数据的预处理 
	a. 多个点云数据的时间同步与融合
	b. 点云数据运动畸变的校正与无效点的处理
	c. 点云的坐标变换与z方向的过滤
	d. 对过滤后的点云进行体素滤波


#### 第11讲 多个激光雷达数据的时间同步与融合

![时间同步](C:/Users/tianc/Desktop/录课/课件/课件markdown/3-2-1-时间同步.jpg)


#### 第12讲 激光雷达数据的运动畸变的校正

激光雷达数据运动畸变的校正, 同时将点云的相对于tracking_frame的点坐标 转成 相对于local slam坐标系的点坐标.

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/3-2-2-运动畸变说明.jpg" alt="3-2-2-运动畸变说明" style="zoom: 50%;" />


#### 第13讲 点云的坐标变换与z轴的过滤

- 初始点云, 点的坐标是相对于tracking_frame的, 点云是围绕着tracking_frame的
  <img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210913210649407.png" alt="image-20210913210649407" style="zoom: 67%;" />


- 运动畸变去除后的点云, 点的坐标相对于local_frame了, 点云依然围绕着tracking_frame
  `sensor::RangefinderPoint hit_in_local = range_data_poses[i] * sensor::ToRangefinderPoint(hit);`

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210913210826651.png" alt="image-20210913210826651" style="zoom:67%;" />



- 以点云的时间(**也就是最后一个点的时间**)预测出来的坐标做为点云的origin

  `accumulated_range_data_.origin = range_data_poses.back().translation()`

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210913212032064.png" alt="image-20210913212032064" style="zoom:67%;" />


- 计算从tracking_frame变换到local_frame原点并且变换后姿态为0的坐标变换
  `transform_to_gravity_aligned_frame = gravity_alignment.cast<float>() * range_data_poses.back().inverse()`

<img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210913212323331.png" alt="image-20210913212323331" style="zoom:67%;" />

- 将点云进行平移与旋转, 点的坐标相对于local_frame, 点云围绕这local_frame坐标系原点
  `sensor::TransformRangeData(range_data, transform_to_gravity_aligned_frame)`

  <img src="C:\Users\tianc\AppData\Roaming\Typora\typora-user-images\image-20210913212753578.png" alt="image-20210913212753578" style="zoom:67%;" />

- 进行z轴的过滤
  `sensor::CropRangeData(sensor::TransformRangeData(range_data, transform_to_gravity_aligned_frame), options_.min_z(), options_.max_z())`

  **单线雷达不能设置 大于0的min_z, 因为单线雷达的z为0** 


#### 第14讲 体素滤波与之后的处理

- 分别对 returns点云 与 misses点云 进行体素滤波

```
sensor::RangeData{ cropped.origin,
     sensor::VoxelFilter(cropped.returns, options_.voxel_filter_size()),
     sensor::VoxelFilter(cropped.misses, options_.voxel_filter_size())};
```

- 对 returns点云 进行自适应体素滤波

`sensor::AdaptiveVoxelFilter(gravity_aligned_range_data.returns, options_.adaptive_voxel_filter_options())`

- 将 原点位于local坐标系原点处的点云 变换成 原点位于匹配后的位姿处的点云

`TransformRangeData(gravity_aligned_range_data, transform::Embed3D(pose_estimate_2d->cast<float>()) )`

- 将 原点位于匹配后的位姿处的点云 返回到node.cc 中, node.cc将这个点云发布出去, 在rviz中可视化



### 6.3 3D情况下的激光雷达数据的预处理

- 进行多个雷达点云数据的时间同步
- 对点云进行第一次体素滤波
- 激光雷达数据运动畸变的校正, 同时将点云的相对于tracking_frame的点坐标 转成 相对于local slam坐标系的点坐标
- 分别对 returns 与 misses 进行第二次体素滤波
- 将原点位于机器人当前位姿处的点云 转成 原点位于local坐标系原点处的点云
- 使用高分辨率进行自适应体素滤波 生成高分辨率点云
- 使用低分辨率进行自适应体素滤波 生成低分辨率点云
- 将 原点位于local坐标系原点处的点云 变换成 原点位于匹配后的位姿处的点云
- 将 原点位于匹配后的位姿处的点云 返回到node.cc 中, node.cc将这个点云发布出去, 在rviz中可视化

## 第七章 初始位姿估计

### 7.1 前端扫描匹配整体函数调用流程分析

### 7.2 基于IMU与里程计的位姿推测器

#### 重要成员变量说明

**姿态预测相关**

- imu_tracker_ 只在添加位姿时更新, 用于保存添加校位姿准时的姿态
- odometry_imu_tracker_ 只在添加位姿时更新, 用于根据里程计数据计算线速度时姿态的预测
- extrapolation_imu_tracker_ 只在添加位姿时更新, 用于位姿预测时的姿态预测

**通过里程计计算的线速度与角速度**

- linear_velocity_from_odometry_ 只在添加里程计数据时更新, 用于位姿预测时的平移量预测
- angular_velocity_from_odometry_ 只在添加里程计数据时更新, 用于不使用imu数据时的**imu_tracker_的角速度**的更新

**通过pose计算的线速度与角速度**

- linear_velocity_from_poses_ 只在添加位姿时更新, 用于位姿预测时 **不使用里程计数据时** 平移量的预测
- angular_velocity_from_poses_ 只在添加位姿时更新, 用于 **不使用里程计数据时** 的**imu_tracker_的角速度**的更新

**传感器数据队列的个数**

- imu_date_ 队列数据的个数最少是1个
- odometry_data_ 队列数据的个数最少是2个, 所以, `odometry_data_.size() < 2` 就意味着不使用里程计
- timed_pose_queue_ 队列数据的个数最少是2个



### 7.3 位姿推测器的优缺点分析与总结

#### 预测位姿时的4种情况 都是匀速模型

- 使用imu, 使用里程计
  - 平移的预测: 通过里程计数据队列开始和末尾的2个数据计算出的线速度乘以时间
  - 姿态的预测: 通过imu的角速度乘以时间

- 使用imu, 不使用里程计
  - 平移的预测: 通过pose数据队列开始和末尾的2个数据计算出的线速度乘以时间
  - 姿态的预测: 通过imu的角速度乘以时间

- 不使用imu, 使用里程计
  - 平移的预测: 通过里程计数据队列开始和末尾的2个数据计算出的线速度乘以时间
  - 姿态的预测: 通过里程计数据队列开始和末尾的2个数据计算出的角速度乘以时间

- 不使用imu, 不是用里程计
  - 平移的预测: 通过pose数据队列开始和末尾的2个数据计算出的线速度乘以时间

  - 姿态的预测: 通过pose数据队列开始和末尾的2个数据计算出的角速度乘以时间

**总结**: 

- 预测平移时: 有里程计就用里程计的线速度, 没有里程计就用pose计算的线速度进行预测
- 预测姿态时: 有IMU就用IMU的角速度, 没有IMU时, 如果有里程计就用里程计计算出的角速度, 没有里程计就用pose计算的角速度进行预测
- 预测的都是相对值, 要加上最后一个pose的位姿



#### 可能有问题的点

- 计算pose的线速度与角速度时, 是采用的数据队列开始和末尾的2个数据计算的
- 计算里程计的线速度与角速度时, 是采用的数据队列开始和末尾的2个数据计算的
- 使用里程计, 不使用imu时, **计算里程计的线速度方向**和**姿态的预测**时, 用的是里程计数据队列开始和末尾的2个数据的平均角速度计算的, **时间长了就不准**
- 不使用里程计, 不使用imu时, 用的是pose数据队列开始和末尾的2个数据的平均角速度计算的, **时间长了就不准**
- **添加位姿时, 没有用pose的姿态对imu_tracker_进行校准, 也没有对整体位姿预测器进行校准, 只计算了pose的线速度与角速度**
- 从代码上看, cartographer认为位姿推测器推测出来的位姿与姿态是准确的

#### 可能的改进建议

- pose的距离越小, 匀速模型越能代替机器人的线速度与角速度, 计算pose的线速度与角速度时, 可以考虑使用最近的2个数据进行计算

- 里程计距离越短数据越准, 计算里程计的线速度与角速度时, 可以考虑使用最近的2个数据进行计算

- 使用里程计, 不使用imu时, 计算里程计的线速度方向时, 可以考虑使用里程计的角度进行计算

- 使用里程计, 不使用imu时, 进行姿态的预测时, 可以考虑使用里程计的角度进行预测

- 不使用里程计, 不使用imu时, 可以考虑用最近的2个pose计算线速度与角速度

- 使用pose对imu_tracker_的航向角进行校准


## 第八章 概率栅格地图

栅格地图是二维激光SLAM的特点, 能够将环境通过地图的形式表达出来.

**栅格地图的实现是二维激光SLAM的一个难点**

三维激光SLAM形成的点云地图不需要自己手动实现点云的数据结构, PCL中有写好的数据类型, 直接调用就行. 视觉SLAM形成的点云地图也可以用PCL来实现

唯独二维激光SLAM的栅格地图需要自己手动实现, 目前所有的二维激光SLAM的栅格地图都是SLAM作者自己写的, 没有通用的数据结构


#### 4.2.4 栅格地图的栅格是如何更新的
<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-1-2-地图更新.jpg" alt="4-1-2-地图更新" style="zoom: 50%;" />

### 8.1 概率栅格地图相关概念简介与公式实现

### 8.2 地图的几个坐标系讲解

**几个地图的坐标系**

```cpp
/**
 * ros的地图坐标系    cartographer的地图坐标系     cartographer地图的像素坐标系 
 * 
 * ^ y                            ^ x              0------> x
 * |                              |                |
 * |                              |                |
 * 0 ------> x           y <------0                y       
 * 
 * ros的地图坐标系: 左下角为原点, 向右为x正方向, 向上为y正方向, 角度以x轴正向为0度, 逆时针为正
 * cartographer的地图坐标系: 坐标系右下角为原点, 向上为x正方向, 向左为y正方向
 *             角度正方向以x轴正向为0度, 逆时针为正
 * cartographer地图的像素坐标系: 左上角为原点, 向右为x正方向, 向下为y正方向
 */
```

**栅格地图的原点的设置**

```cpp
  // Step: 8 将 原点位于local坐标系原点处的点云 变换成 原点位于匹配后的位姿处的点云
  sensor::RangeData range_data_in_local =
      TransformRangeData(gravity_aligned_range_data,
                         transform::Embed3D(pose_estimate_2d->cast<float>()));
  // 将校正后的雷达数据写入submap
  std::unique_ptr<InsertionResult> insertion_result = InsertIntoSubmap(
      time, range_data_in_local, filtered_gravity_aligned_point_cloud,
      pose_estimate, gravity_alignment.rotation());

// 将点云数据写入到submap中
std::vector<std::shared_ptr<const Submap2D>> ActiveSubmaps2D::InsertRangeData(
    const sensor::RangeData& range_data) {
  if (submaps_.empty() ||
      submaps_.back()->num_range_data() == options_.num_range_data()) {
    AddSubmap(range_data.origin.head<2>());
  }

  return submaps();
}

void ActiveSubmaps2D::AddSubmap(const Eigen::Vector2f& origin) {
  if (submaps_.size() >= 2) {
    submaps_.erase(submaps_.begin());
  }
  // 新建一个子图
  submaps_.push_back(absl::make_unique<Submap2D>(
      origin,
      std::unique_ptr<Grid2D>(
          static_cast<Grid2D*>(CreateGrid(origin).release())),
      &conversion_tables_));
}
```

**第一个雷达数据到来时的栅格地图的原点是如何确定的**

```cpp
std::unique_ptr<transform::Rigid2d> LocalTrajectoryBuilder2D::ScanMatch(
    const common::Time time, const transform::Rigid2d& pose_prediction,
    const sensor::PointCloud& filtered_gravity_aligned_point_cloud) {
  if (active_submaps_.submaps().empty()) {
    return absl::make_unique<transform::Rigid2d>(pose_prediction);
  }
  //...
}

std::unique_ptr<LocalTrajectoryBuilder2D::MatchingResult>
LocalTrajectoryBuilder2D::AddAccumulatedRangeData(){
  const transform::Rigid3d non_gravity_aligned_pose_prediction =
      extrapolator_->ExtrapolatePose(time);
  // 将三维位姿先旋转到姿态为0, 再取xy坐标将三维位姿转成二维位姿
  const transform::Rigid2d pose_prediction = transform::Project2D(
      non_gravity_aligned_pose_prediction * gravity_alignment.inverse());
}

// 预测得到time时刻 tracking frame 在 local 坐标系下的位姿
transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  CHECK_GE(time, newest_timed_pose.time);
  // 如果本次预测时间与上次计算时间相同 就不再重复计算
  if (cached_extrapolated_pose_.time != time) {
    // 预测tracking frame在local坐标系下time时刻的位置
    const Eigen::Vector3d translation =
        ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
    // 预测tracking frame在local坐标系下time时刻的姿态
    const Eigen::Quaterniond rotation =
        newest_timed_pose.pose.rotation() *
        ExtrapolateRotation(time, extrapolation_imu_tracker_.get());
    cached_extrapolated_pose_ =
        TimedPose{time, transform::Rigid3d{translation, rotation}};
  }
  return cached_extrapolated_pose_.pose;
}


// 如果Extrapolator没有初始化就进行初始化
void LocalTrajectoryBuilder2D::InitializeExtrapolator(const common::Time time) {
  // 如果已经初始化过了就直接返回
  if (extrapolator_ != nullptr) {
    return;
  }

  // 初始化位姿推测器
  extrapolator_ = absl::make_unique<PoseExtrapolator>(
      ::cartographer::common::FromSeconds(options_.pose_extrapolator_options()
                                              .constant_velocity()
                                              .pose_queue_duration()), // 0.001s
      options_.pose_extrapolator_options()
          .constant_velocity()
          .imu_gravity_time_constant()); // 10
  // 添加初始位姿
  extrapolator_->AddPose(time, transform::Rigid3d::Identity());
}

```



### 8.3 子图与概率地图的实现


#### 4.2.5 cartographer中的栅格地图的相关概念与公式

- **probability**: 栅格被占据的概率

  kMinProbability = 0.1, kMaxProbability = 0.9, kUnknownProbabilityValue = 0

- **Odds**: probability / (1.0f - probability)

- **CorrespondenceCost**: 栅格是free的概率;  CorrespondenceCost + probability  = 1

  kMinCorrespondenceCost = 0.1, kMaxCorrespondenceCost = 0.9

  kUnknownCorrespondenceValue = 0, kUpdateMarker = 32768


- **Value**: 代码里存储的栅格值, 是[0, 32767]范围内的 uint16 整数
- **value_to_correspondence_cost_table_**: 将[0, 1~32767] 映射到 [0, 0.1~0.9] 的转换表
- hit_table_ 计算[0, 1~32767] 按照占用概率0.55更新之后的值
- miss_table_ 计算[0, 1~32767] 按照空闲概率0.49更新之后的值



论文里的地图更新公式讲解


### 8.4 将点云数据写入到栅格地图中

##### 4.2.8.1 查找表

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-2-1-查找表.jpg" alt="4-2-1-查找表" style="zoom: 67%;" />

##### 4.2.8.2 贝汉明算法

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-2-1-贝汉明算法那.jpg" alt="4-2-1-贝汉明算法那" style="zoom: 67%;" />

Bresenham画线算法 [https://www.jianshu.com/p/d63bf63a0e28](https://www.jianshu.com/p/d63bf63a0e28)

图解 cartographer之雷达模型CastRay [https://blog.csdn.net/chaosir1991/article/details/109561010](https://blog.csdn.net/chaosir1991/article/details/109561010)


### 8.5 栅格地图的总结

代码混乱, 计算查找表的代码重复. 看代码时候一定注意自己多写笔记, 把代码结构, 函数调用关系弄懂.

## 第九章 Ceres实现2D扫描匹配

### 9.1 扫描匹配的概念与公式推导


#### 扫描匹配的概念

扫描匹配, 又叫 scan match, 是二维激光SLAM中独有的, 因为只有二维激光SLAM中存在栅格地图. 

**扫描匹配的目的**是找到雷达点云在栅格地图中的位置与角度. 

#### 扫描匹配的实现

**暴力匹配**: 将点云的位置与角度在栅格地图上的每个栅格都获取一下得分, 选出得分最高的一个位置与姿态.

通过点云落在栅格上对应的栅格值的和, 当做这帧点云在这个位姿下的得分.

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-3-1-scan_match.PNG" alt="4-3-1-scan_match" style="zoom: 67%;" />

**基于优化的方法**: 

优化方法: 最速下降法, 高斯牛顿法, LM法

Hector的论文中有公式推导 [A Flexible and Scalable SLAM System with Full 3D Motion Estimation]

Hector论文公式推导与相关代码解析 [https://blog.csdn.net/tiancailx/article/details/113522899](https://blog.csdn.net/tiancailx/article/details/113522899)

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-3-1-scan_match2.PNG" alt="4-3-1-scan_match2" style="zoom:67%;" />

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-3-1-scan_match3.PNG" alt="4-3-1-scan_match3" style="zoom: 67%;" />



#### 扫描匹配的应用

- 前端部分: 存在先验位姿的情况下确定机器人在地图中的位置

- 回环检测: 通过当前点云与之前的地图进行匹配, 如果匹配上了就证明存在回环

- 重定位: 在不存在先验位姿的情况下确定机器人在地图中的位置

  

### 9.2 实时相关性扫描匹配


RealTimeCorrelativeScanMatcher2D  : 就是**暴力搜索**

如何求角度分辨率

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-3-2-相关性扫描匹配.jpg" alt="4-3-2-相关性扫描匹配" style="zoom:50%;" />



### 9.3 Ceres简介


**概念介绍**
[http://www.ceres-solver.org/nnls_tutorial.html#introduction](http://www.ceres-solver.org/nnls_tutorial.html#introduction)

**官方教程**
[http://www.ceres-solver.org/nnls_tutorial.html#hello-world](http://www.ceres-solver.org/nnls_tutorial.html#hello-world)

其他博客: 都是翻译的官方文档
Ceres详解（一）（二）（三）[https://blog.csdn.net/weixin_43991178/article/details/100532618](https://blog.csdn.net/weixin_43991178/article/details/100532618)

**使用步骤**

使用Ceres求解非线性优化问题，一共分为四个步骤

- 第一步 声明CostFunctor


```c++
class MyScalarCostFunctor {
  MyScalarCostFunctor(double k): k_(k) {}

  template <typename T>
  bool operator()(const T* const x , const T* const y, T* e) const {
      // (k - x^T * y)^2, 平方由ceres自动添加
      e[0] = k_ - x[0] * y[0] - x[1] * y[1];
    return true;
  }

 private:
  double k_;
};
```

- 第二步 调用AutoDiffCostFunction函数

```c++
CostFunction* cost_function
    = new AutoDiffCostFunction<MyScalarCostFunctor, 1, 2, 2>(
        new MyScalarCostFunctor(1.0));              ^  ^  ^
                                                    |  |  |
                        Dimension of residual ------+  |  |
                        Dimension of x ----------------+  |
                        Dimension of y -------------------+

CostFunction* cost_function
    = new AutoDiffCostFunction<MyScalarCostFunctor, DYNAMIC, 2, 2>(
        new CostFunctorWithDynamicNumResiduals(1.0),   ^     ^  ^
        runtime_number_of_residuals); <----+           |     |  |
                                           |           |     |  |
                                           |           |     |  |
          Actual number of residuals ------+           |     |  |
          Indicate dynamic number of residuals --------+     |  |
          Dimension of x ------------------------------------+  |
          Dimension of y ---------------------------------------+
```

- 第三步 添加残差块

```c++
ceres::Problem problem;
problem.AddResidualBlock(cost_function);
```

- 第四步 进行求解

```c++
ceres::Solver::Summary summary;
ceres::Solve(ceres_solver_options_, &problem, summary);
std::cout << summary.BriefReport() << std::endl; // summary.FullReport()
```



**class BiCubicInterpolator**

[http://www.ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres19BiCubicInterpolatorE](http://www.ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres19BiCubicInterpolatorE)

看代码



### 9.4 Ceres编程实践



### 9.5 基于优化的扫描匹配


<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-3-3-kpadding.jpg" alt="4-3-3-kpadding" style="zoom:50%;" />




### 9.6 前端扫描匹配的总结

#### 位姿的连续性
RealTimeCorrelativeScanMatcher2D::Match 是通过对像素坐标进行遍历, 所以这个函数计算出的坐标的分辨率是栅格的分辨率, 不是连续的.

CeresScanMatcher2D::Match 优化的是**物理坐标, 不是像素坐标, 所以坐标是连续的.**


#### 可能有问题的点

- 平移和旋转的残差项是逼近于先验位姿的, 当先验位姿不准确时会产生问题

#### 可能的改进建议

- 先将地图的权重调大, 平移旋转的权重调小, 如 1000, 1, 1, 或者 100, 1, 1
- 调参没有作用的时候可以将平移和旋转的残差项注释掉



## 第十章 2D后端优化（上）
### 10.1 后端优化的概念与理论过程分析

### 10.2 后端优化整体函数调用流程分析

### 10.3 添加节点相关函数

### 10.4 子图的保存与任务处理函数

### 10.5 线程池与任务调度


## 第十一章 2D后端优化（中）
### 11.1 子图内约束的计算

### 11.2 子图间约束的计算(回环检测）

### 11.3 使用滑动窗口算法生成多分辨率栅格地图

### 11.4 滑动窗口算法编程实践

### 11.5 分支定界算法简介

### 11.6 基于多分辨率地图的分支定界粗匹配

## 第十二章 2D后端优化（下）
### 12.1 优化时的函数调用流程分析

### 12.2 后端计算位姿的方法总结

### 12.3 约束残差的构建

### 12.4 Landmark残差的构建

### 12.5 里程计与GPS残差的构建

### 12.6 使用Ceres进行全局优化求解

### 12.7 优化后位姿的更新

### 12.8 后端优化部分的总结


## 第十三章 SLAM的结果输出
### 13.1 优化后结果的传递与处理

### 13.2 TF与里程计的发布

### 13.3 轨迹与约束的可视化

### 13.4 ROS地图的发布


#### Cairo简介

Cairo是非常流行的开源2D图形渲染引擎库，它支持包括X-Windos，Win32，图像，pdf在内的各种输出设备。目前，Cairo已被广泛的使用在多个平台上来渲染图形界面，包括Firefox/Webkit-EFL/GTK+/Poppler等等。 

#### 学习资料

Cairo官网 [https://www.cairographics.org/tutorial/](https://www.cairographics.org/tutorial/)
cairo_t相关函数说明(API) [https://www.cairographics.org/manual/cairo-cairo-t.html](https://www.cairographics.org/manual/cairo-cairo-t.html) 
Image Surfaces相关函数说明(API) [https://cairographics.org/manual/cairo-Image-Surfaces.html](https://cairographics.org/manual/cairo-Image-Surfaces.html)

#### 扩展资料

Cairo学习（一） [http://diff3.com/f255/](http://diff3.com/f255/)
Cairo 图形指南 [https://blog.csdn.net/haiwil/category_833019.html](




### 13.5 结束轨迹时的处理

### 13.6 Pbstream文件的保存

## 第十四章 纯定位模式与3D建图
### 14.1 纯定位模式函数调用流程分析

### 14.2 Pbstream文件的加载

### 14.3 子图的修剪

### 14.4 TSDF地图简介

### 14.5 3D网格地图与3D扫描匹配

### 14.6 3D后端优化


## 第十五章 总结与工程化建议
### 15.1 代码的全面总结

### 15.2 Cartographer的优缺点分析 

### 15.3 工程化建议
	a. 提升建图质量
	b. 降低计算量与内存
	a. 纯定位的改进建议






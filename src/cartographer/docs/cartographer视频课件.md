

# cartographer从入门到精通: 原理深剖+源码逐行讲解



## 第一章 编译运行及调参指导

### 1.1 cartographer论文讲解

### 1.2 cartographer的编译

####  1.2.1 官方文档的地址

官方的编译文档的地址为

- https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html
- https://google-cartographer.readthedocs.io/en/latest/index.html

####  1.2.2 依赖的系统环境

- 推荐在ubuntu16.04或者18.04上进行编译
- 推荐使用刚装好的ubuntu系统
- 将ubuntu的软件源设置成清华的或者其他的中国境内的源, 更换完源之后要执行一下 `sudo apt-get update`
- 照着wiki.ros.org安装完对应版本的ROS
- 并在.bashrc中的末尾添加这条句语句, 要把`<distro>`改成对应的ROS版本 
  `source /opt/ros/<distro>/setup.bash`

#####  1.2.3 cartorgapher依赖库的安装

cartorgapher的依赖库可以通过脚本完成一键安装.

##### 第一步: 下载依赖库

在qq群里下载我上传的cartographer_install-20210613.zip, 放到ubuntu系统里并解压

##### 第二步: 打开终端

进入到cartographer_install文件夹, 在文件夹内部空白处单击鼠标右键, 打开终端

##### 第三步 使用脚本完成依赖项的安装

在终端中输入 `./auto-carto-build.sh`

如果提示脚本不是一个可执行的文件, 那就通过 `chmod +x auto-carto-build.sh` 为这个脚本添加可执行权限即可, 之后在通过`./auto-carto-build.sh`执行脚本.

---

##### 解释说明

脚本里进行了4个库的编译与安装

cartographer的依赖库有3个,  分别是abseil-cpp, ceres-solver, protobuf.

在脚本的最后, 也将cartographer安装到系统里了. 安装cartographer后, 就可以在工作空间中只对cartographer_ros进行编译了, 这样修改launch和lua文件后就不需要执行一次编译指令了.

**注意: 如果之前安装过 ceres-solver, protobuf, cartographer 的话, 执行脚本时可能会报有些函数没有定义的错误, 这是由于protobuf或者cartographer不一致版本导致的, 可以先将之前安装的库卸载掉, 再执行安装依赖的脚本**

一定要确保脚本执行之后没有错误输出, 否则之后编译代码会编译不通过.

---

#####  1.2.4 cartorgapher的编译

##### 第一步: 下载注释的代码 

讲师注释的代码的github地址为 
https://github.com/xiangli0608/cartographer_detailed_comments_ws

在某个文件夹内部空白处单击鼠标右键, 打开终端, 输入如下执行进行代码的下载.

`git clone https://github.com/xiangli0608/cartographer_detailed_comments_ws.git`

**注意**, 一定要通过 git clone 的方式下载代码, 因为代码的注释是处于更新状态的, 要定期通过 `git pull origin master` 来更新代码.

要是在github中下载代码的zip压缩文件, 这样下载的代码里不含有.git文件夹, 就不能通过 `git pull` 进行代码的更新了.

如果执行 `git clone` 后没有下载代码, 是由于没有连上github导致的, 可以按 `ctrl c` 中断当前下载, 再次执行 `git clone` , 重复几次试试, 如果还不行, 就在群里问问吧.

##### 第二步: cartorgapher的编译

进入到cartographer_detailed_comments_ws内, 在文件夹内打开终端, 输入
`./catkin_make.sh`

完成对cartographer与cartographer_ros的编译.

注意: 一定要确保之前的依赖项全部安装成功, 否则这里的编译就会报错.

### 1.3 使用讲师的bag运行cartographer

#### 1.3.1 下载数据集

本次使用的bag有2个, 已经上传到qq群文件中, 名字为 bag-2021-06-13.zip , 请提前下载好并放到ubuntu中.

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

### 1.4 配置文件的参数讲解与调参指导

#### 配置文件解读

官网上有一些, 但是不全, 代码里的才是最全最准确的.
https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html

https://google-cartographer.readthedocs.io/en/latest/configuration.html

#### 调参指导

lua文件中坐标系设置的不对, 导致tf树连不起来, 会在运行时产生 `passed to lookupTransform argument source_frame dose not exit` 这种问题.

launch文件中topic名字没设置对, 会在运行时产生 `Queue waiting for data ....` 这种问题.

更具体的请看视频.



## 第二章 cartographer_ros代码讲解

### 看视频的注意事项

- **看视频不等于看懂代码:** 看视频只能大致听懂代码, 不是自己真正理解代码, 光看视频很容易看完就忘
- **要有自己的思考过程: ** 自己读代码 / 自己手动注释代码 / 写技术博客或者笔记
- 尽量不要跳着看视频, 有时跳着看会错过重要知识点, 可以1.25/2.0**倍速观看**
- 可以只听视频, 自己看代码
- 每次听代码讲解的视频之前, 要先将**代码更新**一下

### 第1讲 CMakeLists.txt文件讲解

听不懂没关系, 不影响cartographer的学习

### 第2讲 node_main.cc文件讲解-上


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



### 第8讲 Node类-3

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

### 第9讲 Node类-4

#### 订阅话题的名字对不上时会提示这样的错误信息

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

#### 错误信息截图展示如下

![error_log](C:/Users/tianc/Desktop/录课/课件/课件markdown/2-1-1-error_log.png)

### 第16讲 三维刚体坐标变换 rigid_transform.h

<img src="2-2-1-三维刚体1.jpg" alt="2-2-1-三维刚体1" style="zoom: 33%;" />

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/2-2-2-三维刚体2.jpg" alt="2-2-2-三维刚体2" style="zoom: 33%;" />

### ECEF坐标系

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/2-2-3-ecef.png" alt="2-2-3-ecef.png" style="zoom: 50%;" />

### LaserScan与MultiEchoLaserScan的数据类型

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

### Cairo简介

Cairo是非常流行的开源2D图形渲染引擎库，它支持包括X-Windos，Win32，图像，pdf在内的各种输出设备。目前，Cairo已被广泛的使用在多个平台上来渲染图形界面，包括Firefox/Webkit-EFL/GTK+/Poppler等等。 

#### 学习资料

Cairo官网 [https://www.cairographics.org/tutorial/](https://www.cairographics.org/tutorial/)
cairo_t相关函数说明(API) [https://www.cairographics.org/manual/cairo-cairo-t.html](https://www.cairographics.org/manual/cairo-cairo-t.html) 
Image Surfaces相关函数说明(API) [https://cairographics.org/manual/cairo-Image-Surfaces.html](https://cairographics.org/manual/cairo-Image-Surfaces.html)

#### 扩展资料

Cairo学习（一） [http://diff3.com/f255/](http://diff3.com/f255/)
Cairo 图形指南 [https://blog.csdn.net/haiwil/category_833019.html](



## 第三章 传感器数据的处理

### 3.1 MapBuilder类

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



### 3.2 传感器数据分发器的创建

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



### 3.4 2D场景激光雷达数据的预处理



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



### 3.5 3D场景激光雷达数据的预处理

- 进行多个雷达点云数据的时间同步
- 对点云进行第一次体素滤波
- 激光雷达数据运动畸变的校正, 同时将点云的相对于tracking_frame的点坐标 转成 相对于local slam坐标系的点坐标
- 分别对 returns 与 misses 进行第二次体素滤波
- 将原点位于机器人当前位姿处的点云 转成 原点位于local坐标系原点处的点云
- 使用高分辨率进行自适应体素滤波 生成高分辨率点云
- 使用低分辨率进行自适应体素滤波 生成低分辨率点云
- 将 原点位于local坐标系原点处的点云 变换成 原点位于匹配后的位姿处的点云
- 将 原点位于匹配后的位姿处的点云 返回到node.cc 中, node.cc将这个点云发布出去, 在rviz中可视化



## 第四章 扫描匹配

### 4.1 位姿估计器 PoseExtrapolator

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




### 4.2 概率栅格地图

栅格地图是二维激光SLAM的特点, 能够将环境通过地图的形式表达出来.

**栅格地图的实现是二维激光SLAM的一个难点**

三维激光SLAM形成的点云地图不需要自己手动实现点云的数据结构, PCL中有写好的数据类型, 直接调用就行. 视觉SLAM形成的点云地图也可以用PCL来实现

唯独二维激光SLAM的栅格地图需要自己手动实现, 目前所有的二维激光SLAM的栅格地图都是SLAM作者自己写的, 没有通用的数据结构

#### 4.2.1 ActiveSubmaps2D

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-1-1-submap.jpg" alt="4-1-1-submap" style="zoom: 50%;" />

#### 4.2.2 Submap2D

#### 4.2.3 MapLimits

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





#### 4.2.4 栅格地图的栅格是如何更新的

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-1-2-地图更新.jpg" alt="4-1-2-地图更新" style="zoom: 50%;" />

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



#### 4.2.6 Grid2D

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-1-3-地图增长.jpg" alt="4-1-3-地图增长" style="zoom: 50%;" />

#### 4.2.7 ProbabilityGrid

#### 4.2.8 将雷达数据写成栅格地图

##### 4.2.8.1 查找表

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-2-1-查找表.jpg" alt="4-2-1-查找表" style="zoom: 67%;" />

##### 4.2.8.2 贝汉明算法

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-2-1-贝汉明算法那.jpg" alt="4-2-1-贝汉明算法那" style="zoom: 67%;" />

Bresenham画线算法 [https://www.jianshu.com/p/d63bf63a0e28](https://www.jianshu.com/p/d63bf63a0e28)

图解 cartographer之雷达模型CastRay [https://blog.csdn.net/chaosir1991/article/details/109561010](https://blog.csdn.net/chaosir1991/article/details/109561010)



#### 4.2.9 注意事项

代码混乱, 计算查找表的代码重复. 看代码时候一定注意自己多写笔记, 把代码结构, 函数调用关系弄懂.



### 4.3 扫描匹配的概念与实现

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

  

### 4.4 相关性扫描匹配 

RealTimeCorrelativeScanMatcher2D  : 就是**暴力搜索**

如何求角度分辨率

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-3-2-相关性扫描匹配.jpg" alt="4-3-2-相关性扫描匹配" style="zoom:50%;" />









### 4.5 Ceres简介

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




###  4.6 基于Ceres的扫描匹配

<img src="C:/Users/tianc/Desktop/录课/课件/课件markdown/4-3-3-kpadding.jpg" alt="4-3-3-kpadding" style="zoom:50%;" />



RealTimeCorrelativeScanMatcher2D::Match 是通过对像素坐标进行遍历, 所以这个函数计算出的坐标的分辨率是栅格的分辨率, 不是连续的.

CeresScanMatcher2D::Match 优化的是**物理坐标, 不是像素坐标, 所以坐标是连续的.**

#### 可能有问题的点

- 平移和旋转的残差项是逼近于先验位姿的, 当先验位姿不准确时会产生问题

#### 可能的改进建议

- 先将地图的权重调大, 平移旋转的权重调小, 如 1000, 1, 1, 或者 100, 1, 1
- 调参没有作用的时候可以将平移和旋转的残差项注释掉



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








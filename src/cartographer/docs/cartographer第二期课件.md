

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
### 1.5 参数的详解与调参总结

## 第二章 Cartographer_ros代码入门
### 2.1 VScode的配置与使用
### 2.2 Cartographer_ros的功能与框架分析
### 2.3 CMakeLists.txt文件讲解
### 2.4 Main函数使用的依赖库简介
### 2.5 Main函数讲解
### 2.6 配置文件的加载

## 第三章 Cartographer_ros代码进阶
### 3.1 开始轨迹的相关处理
### 3.2 数据采样器
### 3.3 订阅话题与注册回调函数
### 3.4 传感器数据的走向与传递


## 第四章 传感器的格式转换与类型变换
### 4.1 三维刚体坐标变换的理论与实现
### 4.2 IMU数据、里程计数据与Landmark数据的格式转换与坐标变换
### 4.3 GPS数据的处理与编程实践
### 4.4 雷达数据的格式转换与坐标变换
### 4.5 Cartographer_ros中的传感器数据的传递过程总结

## 第五章 传感器数据的分发处理
### 5.1 Cartographer中的传感器数据分发过程分析
### 5.2 数据分发器相关类的构造与初始化
### 5.3 顺序多队列与数据的分发处理
### 5.4 阻塞队列与生产者消费者模式
### 5.5 传感器数据的分发处理过程总结

## 第六章 点云数据的预处理
### 6.1 传感器数据的走向
### 6.2. 2D情况下激光雷达数据的预处理 
	a. 多个点云数据的时间同步与融合
	b. 点云数据运动畸变的校正与无效点的处理
	c. 点云的坐标变换与z方向的过滤
	d. 对过滤后的点云进行体素滤波
### 6.3 3D情况下的激光雷达数据的预处理

## 第七章 初始位姿估计
### 7.1 前端扫描匹配整体函数调用流程分析
### 7.2 基于IMU与里程计的位姿推测器
	a. 进行线速度与角速度的预测
	b. 根据匹配后的位姿进行状态量的更新
	c. 位置与姿态的预测
### 7.3 位姿推测器的优缺点分析与总结

## 第八章 概率栅格地图
### 8.1 概率栅格地图相关概念简介与公式实现
### 8.2 地图的几个坐标系讲解
### 8.3 子图与概率地图的实现
### 8.4 将点云数据写入到栅格地图中
### 8.5 栅格地图的总结

## 第九章 Ceres实现2D扫描匹配
### 9.1 扫描匹配的概念与公式推导
### 9.2 实时相关性扫描匹配
### 9.3 Ceres简介
### 9.4 Ceres编程实践
### 9.5 基于优化的扫描匹配
### 9.6 前端扫描匹配的总结

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






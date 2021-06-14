# cartographer 超详细注释代码

## 项目简介
基于2021.04.20日在github上下载的master的代码的注释版本
1. 存在规范、超详细的注释
**注释多数在 .cc 文件中, .h文件中存在cartographer原本的注释**
2. 增加关键地方的公式推导
3. 难理解部分代码画图展示原理
4. c++11新标准的语法的标注与简介

## 课程简介与购买链接
[课程简介与购买链接](https://mp.weixin.qq.com/s?__biz=MzIxOTczOTM4NA==&mid=2247519351&idx=1&sn=870a6b1eddf74e51506d2e3af87e2ed1&chksm=97d469e0a0a3e0f6c568686ffa44d52edeee171c751c87b4fca2066d42ed31def2dd1e992698&mpshare=1&scene=1&srcid=06138WbGlINKf3uMM9KaVDi8&sharer_sharetime=1623575086288&sharer_shareid=e0bf4e1cc54f09628a44697dfe50325e&exportkey=AR7RZxSncUsVlZg4gSHEVPw%3D&pass_ticket=eslcXL6f%2BHecxgumHaP%2BPfhvfGtlNNZfPvtBZvQmODNPJP5LT2Stt5%2FM07etmy1a&wx_header=0#rd)

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


## 相关命令

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

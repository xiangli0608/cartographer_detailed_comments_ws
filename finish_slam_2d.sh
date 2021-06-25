#!/bin/bash

source install_isolated/setup.bash

map_dir="${HOME}/carto_ws/map"
map_name="2d-1"

# 检查文件夹是否存在, 如果不存在就创建文件夹
if [ ! -d "$map_dir" ];then
  echo "文件夹不存在, 正在创建文件夹"
  mkdir -p $map_dir
fi


# finish slam
rosservice call /finish_trajectory 0

# make pbstream
rosservice call /write_state "{filename: '$map_dir/$map_name.pbstream'}"

# pbstream to map
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
-pbstream_filename=$map_dir/$map_name.pbstream \
-map_filestem=$map_dir/$map_name

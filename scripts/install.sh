#!/bin/bash -e

working_dir=${PWD}

if [ -d "./catkin_ws" ]; then
  working_dir=$(realpath .)
elif [ -d "../catkin_ws" ]; then
  working_dir=$(realpath ..)
else
  echo "You're running the script from an invalid dir"
  echo "Run it from the base of the wifi-dataset repo"
  exit 1
fi


cd $working_dir

git submodule update --init

cd ./tools/protobuf
git checkout tags/v3.4.1
mkdir -p build
cd build
cmake -G Ninja \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -Dprotobuf_BUILD_TESTS=OFF \
  ../cmake
ninja
sudo ninja install


source /opt/ros/indigo/setup.bash

sudo apt-get update && sudo apt-get install -y \
  python-wstool python-rosdep ninja-build

sudo rm /etc/ros/rosdep/sources.list.d/20-default.list;

cd $working_dir
sudo rosdep init && rosdep update;
rosdep install --from-paths carto_ws/src catkin_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y;

cd $working_dir
source ./carto_ws/install_isolated/setup.sh
cd carto_ws && \
  catkin_make_isolated --install --use-ninja

cd $working_dir
cd catkin_ws && \
  catkin_make





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

source /opt/ros/indigo/setup.bash

sudo apt-get update && sudo apt-get install -y \
  python-wstool python-rosdep ninja-build

mkdir -p carto_ws/src && \
  wstool init carto_ws/src env/carto.rosinstall

wstool update -t carto_ws/src

./carto_ws/src/cartographer/scripts/install_proto3.sh

sudo rm /etc/ros/rosdep/sources.list.d/20-default.list;

sudo rosdep init && rosdep update;
rosdep install --from-paths carto_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y;

cd carto_ws && \
  catkin_make_isolated --install --use-ninja



rosdep install --from-paths catkin_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y;

cd catkin_ws && \
  catkin_make install





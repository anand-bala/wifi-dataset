#!/bin/bash


if [ -d "./catkin_ws" ]; then
   cd .
 elif [ -d "../catkin_ws" ]; then
   cd ..
 else
   echo "You're running the script from an invalid dir"
   echo "Run it from the base of the wifi-dataset repo"
   exit 1
 fi

source /opt/ros/indigo/setup.bash
git submodule update --init


sudo apt-get update && apt-get install -y \
  python-wstool python-rosdep ninja-build \
  readline

mkdir -p carto_ws/src && \
  wstool init carto_ws/src env/carto.rosinstall

./carto_ws/src/cartographer/scripts/install_proto3.sh

sudo rm /etc/ros/rosdep/sources.list.d/20-default.list;

sudo rosdep init && rosdep update;
rosdep install --from-paths carto_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y;

cd carto_ws && \
  catkin_make_isolated --install --use-ninja


rosdep install --from-paths catkin_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y;

cd catkin_ws && \
  source /opt/ros/indigo/setup.bash && \
  catkin_make install





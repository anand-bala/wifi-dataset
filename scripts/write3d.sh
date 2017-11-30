#!/bin/bash

if [ "$#" -lt 2 ]; then
	echo "Too few arguments."
	echo "Usage: $0 <pbstream assets> <bag file>"
	exit 1
fi

ws1=""
ws2=""
if [ -d "./catkin_ws" ] && [ -d "./carto_ws" ]; then
	ws1=`realpath ./catkin_ws`
	ws2=`realpath ./carto_ws`
elif [ -d "../catkin_ws" ] && [ -d "../carto_ws" ]; then
	ws1=`realpath ../catkin_ws`
	ws2=`realpath ../carto_ws`
else
	echo "There seems to be some problem!"
	echo "The workspace either has not been initialized correctly"
	echo "Or you are not in the workspace directory"
	exit 1
fi

echo "Sourcing the following ROS workspaces"
echo $ws1
echo $ws2

src1=""
src2=""

if [ -d "$ws1/install_isolated" ]; then
	src1="$ws1/install_isolated"
elif [ -d "$ws1/install" ]; then
	src1="$ws1/install"
elif [ -d "$ws1/devel_isolated" ]; then
	src1="$ws1/devel_isolated"
elif [ -d "$ws1/devel" ]; then
	src1="$ws1/devel"
else
	echo "Please build the workspace in: $ws1"
	exit 1
fi

if [ -d "$ws2/install_isolated" ]; then
	src2="$ws2/install_isolated"
elif [ -d "$ws2/install" ]; then
	src2="$ws2/install"
elif [ -d "$ws2/devel_isolated" ]; then
	src1="$ws2/devel_isolated"
elif [ -d "$ws2/devel" ]; then
	src1="$ws2/devel"
else
	echo "Please build the workspace in: $ws1"
	exit 1
fi

source $src2/setup.sh
source $src1/setup.sh --extend

echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}"

roslaunch dataset_ros assets_writed_turtlebot_3d.launch \
	bag_filenames:=`realpath $2` \
	pose_graph_filename:=`realpath $1`



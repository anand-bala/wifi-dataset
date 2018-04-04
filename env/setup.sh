#!/bin/bash

ws1=""
ws2=""
if [ -d "./catkin_ws" ] && [ -d "./carto_ws" ]; then
  ws1=./catkin_ws
  ws2=./carto_ws
  export DATASET_WS=`realpath .`
elif [ -d "../catkin_ws" ] && [ -d "../carto_ws" ]; then
  ws1=../catkin_ws
  ws2=../carto_ws
  export DATASET_WS=`realpath ..`
else
  echo "There seems to be some problem!"
  echo "The workspace either has not been initialized correctly"
  echo "Or you are not in the workspace directory"
  return
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
  return
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
  echo "Please build the workspace in: $ws2"
  return
fi
source $src2/setup.sh
source $src1/setup.sh --extend



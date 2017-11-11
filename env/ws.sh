#!/bin/bash

function turtlebot_master {
  export ROS_MASTER_URI="http://10.42.0.1:11311"
}

function ws_source {
  local ws1=""
  local ws2=""
  if [ -d "./catkin_ws" ] && [ -d "./carto_ws" ]; then
    local ws1=`realpath ./catkin_ws`
    local ws2=`realpath ./carto_ws`
  elif [ -d "../catkin_ws" ] && [ -d "../carto_ws" ]; then
    local ws1=`realpath ../catkin_ws`
    local ws2=`realpath ../carto_ws`
  else
    echo "There seems to be some problem!"
    echo "The workspace either has not been initialized correctly"
    echo "Or you are not in the workspace directory"
    return
  fi
  
  echo "Sourcing the following ROS workspaces"
  echo $ws1
  echo $ws2
  local src1=""
  local src2=""
  if [ -d "$ws1/install_isolated" ]; then
    local src1="$ws1/install_isolated"
  elif [ -d "$ws1/install" ]; then
    local src1="$ws1/install"
  elif [ -d "$ws1/devel_isolated" ]; then
    local src1="$ws1/devel_isolated"
  elif [ -d "$ws1/devel" ]; then
    local src1="$ws1/devel"
  else
   echo "Please build the workspace in: $ws1"
   return
  fi
  if [ -d "$ws2/install_isolated" ]; then
    local src2="$ws2/install_isolated"
  elif [ -d "$ws2/install" ]; then
    local src2="$ws2/install"
  elif [ -d "$ws2/devel_isolated" ]; then
    local src1="$ws2/devel_isolated"
  elif [ -d "$ws2/devel" ]; then
    local src1="$ws2/devel"
  else
   echo "Please build the workspace in: $ws1"
   return
  fi
  source $src2/setup.sh
  source $src1/setup.sh --extend

}



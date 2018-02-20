#!/bin/bash

ls /data/

if [ "$#" -ne 1 ]; then
  echo "Incorrect usage."
  echo "Pass bagfile as argument"
  exit 1;
fi

source /opt/ros/indigo/setup.bash
source env/ws.sh

bagfile=`realpath $1`

roslaunch dataset_ros offline_turtlebot2d.launch bag_filenames:=$realpath


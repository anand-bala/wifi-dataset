#!/bin/bash

function myreadlink() {
  (
  cd $(dirname $1)         # or  cd ${1%/*}
  echo $PWD/$(basename $1) # or  echo $PWD/${1##*/}
  )
}


ls /data

if [ "$#" -ne 1 ]; then
  echo "Incorrect usage."
  echo "Pass bagfile as argument"
  exit 1;
fi

pwd


source /opt/ros/indigo/setup.bash
source env/setup.sh

bagfile=`myreadlink $1`

roslaunch dataset_ros offline_turtlebot2d.launch bag_filenames:=$bagfile


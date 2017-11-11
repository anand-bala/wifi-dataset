#!/bin/bash

if [ "$#" -ne "2" ]; then
  echo "Please pass target directory and prefix"
  exit 1
fi

target_dir=`realpath $1`
prefix=$2

echo "Target directory: $target_dir"
echo "Rosbag prefix:    $prefix"
echo "$target_dir/${prefix}.bag"

rosbag record -O $target_dir/${prefix}.bag \
	/APInfo \
	/camera/depth_registered/image_raw \
	/camera/rgb/camera_info \
	/camera/rgb/image_color \
	/camera/rgb/image_raw \
	/camera/rgb/image_rect_color \
	/odom \
	/tf \
	/velodyne_points \
	/mobile_base/sensors/imu_data_raw


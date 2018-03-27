#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "usage: $0 <session-dir>"
  exit 1
fi



sessions_dir=`realpath $1`
orig_dir="${sessions_dir/sessions/tmp}"
output_file="${sessions_dir/sessions/outputs}/trajectory.pbstream"
output_dir=$(dirname "${output_file}")
sessions=$(ls -d $sessions_dir/*.bag)

echo "Correcting Timestamps in $sessions_dir "
echo "Creating Directory: $orig_dir"
echo "Output File: ${output_file}"
sudo mkdir -v -p $orig_dir
sudo chown $(whoami) $orig_dir
sudo chmod 0775 $orig_dir
sudo mkdir -v -p $output_dir 
sudo chown $(whoami) $output_dir
sudo chmod 0775 $output_dir

if [ -d "./scripts" ]; then
  working_dir=$(realpath .)
elif [ -d "../scripts" ]; then
  working_dir=$(realpath ..)
else
  echo "You're running the script from an invalid dir"
  echo "Run it from the base of the wifi-dataset repo"
  exit 1
fi


cd $working_dir

./scripts/correctBags.py --output-bag $orig_dir/output.bag $sessions
./scripts/ts1.py $orig_dir/output.bag
rosbag info $orig_dir/output.bag > $orig_dir/info.yaml

source ./env/setup.sh
roslaunch dataset_ros offline_turtlebot2d.launch bag_filenames:=$orig_dir/output.bag

mv -v -f $orig_dir/output.bag.pbstream $output_file
mv -v -f $orig_dir/info.yaml $output_dir/
roslaunch dataset_ros dump_trajectory.launch pbfile:=$output_file outfile:=$output_dir/trajectory.csv




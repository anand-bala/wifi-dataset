#!/usr/bin/env python2

import sys
import rosbag
import time
import subprocess
import yaml
import rospy
import os
import argparse
import math
from shutil import move

import sensor_msgs
import sensor_msgs.point_cloud2 as pc2

CONFIG = {
    'laserscan': {
      'transform_tolerance':  0.01,
      'min_height':           -0.3,
      'max_height':           0.3,
      'angle_min':            -3.14,
      'angle_max':            3.14,
      'angle_increment':      0.0087, # M_PI/360.0
      'scan_time':            0.3333,
      'range_min':            0.05,
      'range_max':            100.0,
      'use_inf':              True,
      'inf_epsilon':          1.0,
      },
    'max_offset': 2.0,
    'topics': [
        '/tf', '/scan', '/imu', '/odom',
        '/mobile_base/sensors/imu_data_raw', '/velodyne_points',
        ],
    }


def status(bag, length, percent):
  sys.stdout.write('\x1B[2K') # Erase entire current line
  sys.stdout.write('\x1B[0E') # Move to the beginning of the current line
  progress = bag + " ["
  for i in range(0, length):
    if i < length * percent:
      progress += '='
    else:
      progress += ' '
  progress += "] " + str(round(percent * 100.0, 2)) + "%"
  sys.stdout.write(progress)
  sys.stdout.flush()


def main(args):
  parser = argparse.ArgumentParser(description='Clean up bags for Dataset.')
  parser.add_argument('bagfiles', nargs='+', help='input bag files')
  parser.add_argument('--max-offset', nargs=1, help='max time offset (sec) to correct.', default='2', type=float)
  parser.add_argument('--output-bag', nargs=1, help='output file for bags.', default='scans.bag')
  args = parser.parse_args()

  with rosbag.Bag(args.output_bag, 'w') as outbag:
    n_bags = len(args.bagfiles)
    for bagfile in args.bagfiles:
      correctBag(bagfile, outbag)


def correctBag(bagfile, outbag):
  # Get bag duration  
  info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', bagfile], stdout=subprocess.PIPE).communicate()[0])
  duration = info_dict['duration']
  start_time = info_dict['start']

  last_time = time.clock()
  for topic, msg, t in rosbag.Bag(bagfile).read_messages():
    if time.clock() - last_time > .1:
      percent = (t.to_sec() - start_time) / duration
      status(bagfile, 40, percent)
      last_time = time.clock()

    if topic not in CONFIG['topics']:
        continue
    # This also replaces tf timestamps under the assumption 
    # that all transforms in the message share the same timestamp
    if topic == "/tf" and msg.transforms:
      # Writing transforms to bag file 1 second ahead of time to ensure availability
      diff = math.fabs(msg.transforms[0].header.stamp.to_sec() - t.to_sec())
      outbag.write(topic, msg, msg.transforms[0].header.stamp - rospy.Duration(1) if diff < CONFIG['max_offset'] else t)
    elif msg._has_header:
      diff = math.fabs(msg.header.stamp.to_sec() - t.to_sec())
      topic, msg = correctMsg(topic, msg)
      outbag.write(topic, msg, msg.header.stamp if diff < CONFIG['max_offset'] else t)
    else:
      outbag.write(topic, msg, t)
  status(bagfile, 40, 1)
  print "\ndone"


def correctMsg(topic, msg):
  if topic == "/mobile_base/sensors/imu_data_raw":
    return correctImu(msg)
  elif topic == "/velodyne_points":
    return getLaserScan(msg)
  else:
    return topic, msg


def correctImu(msg):
  imu_out = msg
  imu_out.linear_acceleration.x = 0;
  imu_out.linear_acceleration.y = 0;
  imu_out.linear_acceleration.z = 9.8;
  return "/imu", imu_out


def getLaserScan(msg):
  scan = sensor_msgs.msg.LaserScan()
  scan.header = msg.header
  conf = CONFIG['laserscan']
  if 'target_frame' in conf.keys():
    scan.header.frame_id = conf['target_frame']
  scan.angle_min = conf['angle_min']
  scan.angle_max = conf['angle_max']
  scan.angle_increment = conf['angle_increment']
  scan.time_increment = 0.0
  scan.scan_time = conf['scan_time']
  scan.range_min = conf['range_min']
  scan.range_max = conf['range_max']

  range_size = int(math.ceil((scan.angle_max - scan.angle_min) / scan.angle_increment))
  scan.ranges = [float('Inf') if conf.get('use_inf', False) else scan.range_max+conf['inf_epsilon']]*range_size
  # TODO(anand): Incorporate TF
  for p in pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
    x,y,z = p[0], p[1], p[2]
    if not conf['min_height'] <= z <= conf['max_height']:
      continue
    r = math.hypot(x,y)
    if r < scan.range_min:
      continue
    angle = math.atan2(x,y)
    if not scan.angle_min <= angle <= scan.angle_max:
      continue
    i = int((angle - scan.angle_min) / scan.angle_increment);
    if r < scan.ranges[i]:
      scan.ranges[i] = r
    return '/scan', scan



if __name__ == "__main__":
  main(sys.argv[1:])



#!/usr/bin/env python

import time

import argparse
import rosbag
import rospy


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Change Timestamp of rosbag')
    parser.add_argument('input')
    parser.add_argument('output')
    parser.add_argument('-t', '--time', help='initial timestamp')

    args = parser.parse_args()

    begin_time = rospy.Time.from_sec(time.time())
    if args.time:
        begin_time = rospy.Time.from_sec(time.strptime(args.time, '%d %b %Y %H:%M:%S'))

    print begin_time



#!/usr/bin/env python

import time

import argparse
import rosbag
import rospy

import os
import logging
import yaml

def check(args):
    if not os.path.isfile(args.input):
        logging.error("Input bagfile doesn't exist")
        return None
    if not os.path.isdir(os.path.dirname(args.output)):
        try:
            os.makedirs(os.path.dirname(args.output))
        except OSError as exc: 
            if exc.errno == errno.EEXIST and os.path.isdir(directory_name):
                pass 
    return (os.path.abspath(args.input), os.path.abspath(args.output))


def fix(input_bag, output_bag, begin_time):
    info_dict = yaml.load(Bag(input_bag, 'r')._get_yaml_info())
    start = rospy.Time.from_sec(info_dict.start)
    with rosbag.Bag(output_bag,'w') as out_bag:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            diff_m = t - start
            diff_h = msg.header.stamp - start
            tm_ = begin_time + diff_m
            th_ = begin_time + diff_h
            msg.header.stamp = th_
            out_bag.write(topic, msg, tm_) 

    return 0

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Change Timestamp of rosbag')
    parser.add_argument('input')
    parser.add_argument('output')
    parser.add_argument('-t', '--time', help='initial timestamp')

    args = parser.parse_args()

    begin_time = rospy.Time.from_sec(time.time())
    if args.time:
        begin_time = rospy.Time.from_sec(time.strptime(args.time, '%d %b %Y %H:%M:%S'))

    logging.info("Adjusting timestamps to begin at: %f", begin_time)

    bags = check(args)
    if bags is None:
        return 1

    return fix(*bags, begin_time)



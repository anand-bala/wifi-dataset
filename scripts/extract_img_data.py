#!/usr/bin/env python
from __future__ import print_function

from datetime import datetime
import os
import sys

import rosbag
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def mkdir_p(path):
    if not os.path.exists(path):
        os.makedirs(path)


class Extractor:
    def __init__(self, output_dir):
        # type: (str)
        self.output_dir = os.path.abspath(output_dir)

        self.bridge = CvBridge()

    def image_cb(self, topic, data, t):
        # type: (str, Image, rospy.Time) -> None

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
            sys.exit(1)

        outfile = os.path.join(self.output_dir,
                               datetime.fromtimestamp(t.to_sec()).strftime('%Y-%m-%d'),
                               topic.strip("/"), str(t.secs), str(t) + '.png')
        outdir = os.path.dirname(outfile)
        mkdir_p(outdir)
        cv2.imwrite(outfile, cv_image)


def main(output_dir, input_files):
    # type: (str, list[str]) -> None
    ex = Extractor(output_dir)
    for bagfile in input_files:
        print('Extracting images from: {}'.format(bagfile))
        bag = rosbag.Bag(bagfile)
        for topic, msg, t in bag.read_messages():
            if msg._type == 'sensor_msgs/Image':
                ex.image_cb(topic, msg, t)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Extract raw data from bag file')
    parser.add_argument('bagfiles', nargs='+', help='input bag files')
    parser.add_argument('-o', '--output', help='output file for bags.', default='raw_data')
    args = parser.parse_args()

    print(args.output)
    print(args.bagfiles)
    main(args.output, args.bagfiles)

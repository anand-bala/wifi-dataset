#!/usr/bin/env python

import struct
import rospy
from std_msgs.msg import String
from manual_measurements.msg import AccessPointInfo
from manual_measurements.msg import WifiInfo

import config
import re
import subprocess

cellNumberRe = re.compile(r"^Cell\s+(.+)\s+-\s+Address:\s(.+)$")
regexps = [
    re.compile(r"^ESSID:\"(.*)\"$"),
#    re.compile(r"^Protocol:(.+)$"),
#    re.compile(r"^Mode:(.+)$"),
    re.compile(r"^Frequency:([\d.]+) (.+) \(Channel (\d+)\)$"),
#    re.compile(r"^Encryption key:(.+)$"),
    re.compile(r"^Quality=(\d+)/(\d+)\s+Signal level=(.+) d.+$"),
    re.compile(r"^Extra: Last beacon: (\d+)ms ago$"),
]


class AccessPointsReader:

    def __init__(self):
        rospy.init_node('AccessPointsReader', anonymous=True)
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher('APInfo2', WifiInfo, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            message = WifiInfo()
            message.header.stamp = rospy.Time.now()
            message.accesspoint = self.getAccessPoints()
            self.pub.publish(message)
            self.rate.sleep()

    def fixStrength(self, val):
        return struct.unpack('B', val)[0]

    def parse(self, content):
        cells = []
        lines = content.split('\n')
	for line in lines:
	    line = line.strip()
            cellNumber = cellNumberRe.search(line)
            if cellNumber is not None:
                cells.append(AccessPointInfo())
		cells[-1].addr = cellNumber.group(2)
		continue                
	    i = 0
            for expression in regexps:
                result = expression.search(line)
		if result is not None:
		    if i==0:
			cells[-1].name = str(result.group(1))
		    elif i==1:
			cells[-1].frequency = int(float(result.group(1)))
		    elif i==2:
			cells[-1].strength = int(result.group(1))
			cells[-1].strength_total = int(result.group(2))
			cells[-1].signal_level = int(result.group(3))
		    elif i==3:
			cells[-1].last_check = result.group(1)
		    continue
		i = i + 1
        return cells
    
    def getAccessPoints(self):
        aps = []
        try:
            cmd = ["sudo", "iwlist", "wlan0", "scan"]
	    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            points = proc.stdout.read().decode('utf-8')

            aps = self.parse(points)
        except Exception:
            pass
        return aps

def main():
    AccessPointsReader().run()

if __name__ == "__main__":
    main()

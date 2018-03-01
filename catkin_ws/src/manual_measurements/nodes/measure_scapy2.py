#!/usr/bin/env python

import struct
import rospy
from std_msgs.msg import String
from manual_measurements.msg import AccessPointInfo
from manual_measurements.msg import WifiInfo
from scapy.all import *

import config
import re
import subprocess

cellNumberRe = re.compile(r"^Cell\s+(.+)\s+-\s+Address:\s(.+)$")
regexp1 = re.compile(r"^ESSID:\"(.*)\"$")
#    re.compile(r"^Protocol:(.+)$"),
#    re.compile(r"^Mode:(.+)$"),
regexp2 = re.compile(r"^Frequency:([\d.]+) (.+) \(Channel (\d+)\)$")
#    re.compile(r"^Encryption key:(.+)$"),
regexp3 = re.compile(r"^Quality=(\d+)/(\d+)\s+Signal level=(.+) d.+$")
regexp4 = re.compile(r"^Extra: Last beacon: (\d+)ms ago$")



class AccessPointsReader:

    def __init__(self):
        rospy.init_node('AccessPointsReader', anonymous=True)
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher('APInfo', WifiInfo, queue_size=10)
	self.message = WifiInfo()
	self.count = 0
	self.cells = []

    def run(self):
        while not rospy.is_shutdown():
	    sniff(iface="wlan1", prn = self.PacketHandler)
            
            

    def fixStrength(self, val):
        return struct.unpack('B', val)[0]
    def PacketHandler(self, pkt):
	if pkt.haslayer(Dot11) :
	    if pkt.type == 0 and pkt.subtype == 8 :
	      if pkt.haslayer(Dot11Beacon) or pkt.haslayer(Dot11ProbeResp):
		try:
		    extra = pkt.notdecoded
		    rssi = -(256-ord(extra[-4:-3]))
		except:
		    rssi = -100
		if ((self.count % 10)==0):
		    self.message.header.stamp = rospy.Time.now()
		    self.message.accesspoint = self.cells
		    self.pub.publish(self.message)
		    self.rate.sleep()
		    self.message = WifiInfo()
		    del self.cells[:]
		    self.cells = []
		self.count = self.count + 1;
		self.cells.append(AccessPointInfo())
		self.cells[-1].addr = pkt.addr2	
		self.cells[-1].signal_level = int(rssi)
		self.cells[-1].name = pkt.info
		
		
    def parse(self, content):
        cells = []
        lines = content.split('\n')
	for i in range(0, len(lines)):
	    line = lines[i]
            line = line.strip()
	    cellNumber = cellNumberRe.search(line)
            if cellNumber is not None:
                cells.append(AccessPointInfo())
                cells[-1].addr = cellNumber.group(2)
	    	print cells[-1].addr
                continue                
            result = regexp1.search(line)
            if result is not None:
	    	cells[-1].name = str(result.group(1))
	    result = regexp2.search(line)
            if result is not None:
            	cells[-1].frequency = int(float(result.group(1)))
            result = regexp3.search(line)
            if result is not None:
	    	#print int(result.group(1))
	    	#print int(result.group(2))
	    	#print int(result.group(3))
                cells[-1].strength = int(result.group(1))
                cells[-1].strength_total = int(result.group(2))
                cells[-1].signal_level = int(result.group(3))
            result = regexp4.search(line)
            if result is not None:
		cells[-1].last_check = result.group(1)
		
	         
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

#!/usr/bin/env python

import struct
import rospy
import NetworkManager
from std_msgs.msg import String
from manual_measurements.msg import AccessPointsInfo

import config

class AccessPointsReader:

    def __init__(self):
        rospy.init_node('AccessPointsReader', anonymous=True)
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher('AccessPointsInfo', AccessPointsInfo, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            message = AccessPointsInfo()
            message.header.stamp = rospy.Time.now()
            message.accesspoints.data = self.getAccessPoints()
            self.pub.publish(message)
            self.rate.sleep()

    def fixStrength(self, val):
        return struct.unpack('B', strength)[0]

    def getAccessPoints(self):
        params = ""
        try:
            for dev in NetworkManager.NetworkManager.GetDevices():
                if dev.DeviceType != NetworkManager.NM_DEVICE_TYPE_WIFI:
                    continue
                aps = [ap for ap in dev.SpecificDevice().GetAccessPoints()]
                for ap in sorted(aps, key=lambda ap: ap.Ssid):
                    params += "({0};{1};{2}),".format(ap.Ssid, ap.HwAddress, ap.Strength if not config.convertHWAddress else self.fixStrength(ap.Strength))
            params = params[:-1]
        except Exception:
            pass
        return params

def main():
    AccessPointsReader().run()

if __name__ == "__main__":
    main()

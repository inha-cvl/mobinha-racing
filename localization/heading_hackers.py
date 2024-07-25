#!/usr/bin/env python

import rospy
from ublox_msgs.msg import NavPVT
class NAVPVT:

    def __init__(self):
        rospy.init_node("hackers")
        self.pub = rospy.Publisher("/ublox/navpvt", NavPVT, queue_size=1)
        self.hacked_msg = NavPVT()
        self.cnt = 0

    def callback(self, msg):
        self.hacked_msg.condition = True
        self.hacked_msg.iTOW = msg.iTOW
        self.hacked_msg.year = msg.year
        self.hacked_msg.month = msg.month
        self.hacked_msg.day = msg.day
        self.hacked_msg.hour = msg.hour
        self.hacked_msg.min = msg.min
        self.hacked_msg.sec = msg.sec
        self.hacked_msg.valid = msg.valid
        self.hacked_msg.tAcc = msg.tAcc
        self.hacked_msg.nano = msg.nano
        self.hacked_msg.fixType = msg.fixType
        self.hacked_msg.flags = msg.flags
        self.hacked_msg.flags2 = msg.flags2
        self.hacked_msg.numSV = msg.numSV
        self.hacked_msg.lon = msg.lon
        self.hacked_msg.lat = msg.lat
        self.hacked_msg.height = msg.height
        self.hacked_msg.hMSL = msg.hMSL
        self.hacked_msg.hAcc = msg.hAcc
        self.hacked_msg.vAcc = msg.vAcc
        self.hacked_msg.velN = msg.velN
        self.hacked_msg.velE = msg.velE
        self.hacked_msg.velD = msg.velD
        self.hacked_msg.gSpeed = msg.gSpeed
        self.hacked_msg.heading = msg.heading*1e-5
        self.hacked_msg.sAcc = msg.sAcc
        self.hacked_msg.headAcc = msg.headAcc
        self.hacked_msg.pDOP = msg.pDOP
        self.hacked_msg.reserved1 = msg.reserved1
        self.hacked_msg.headVeh = msg.headVeh
        self.hacked_msg.magDec = msg.magDec
        self.hacked_msg.magAcc = msg.magAcc

    def hack_heading(self):
        self.hacked_msg.heading = -179.0
        self.pub.publish()
        self.cnt+=1

    def main(self):
        while not rospy.is_shutdown():
            _in = input("press sth: ")
            # print(_in)
            if _in == "":
                # print("asdf")
                self.hack_heading()

if __name__ == "__main__":
    navpvt = NAVPVT()
    navpvt.main()
        
            

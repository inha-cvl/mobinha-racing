#!/usr/bin/env python

import rospy
from ublox_msgs.msg import NavPVT
import threading
from std_msgs.msg import Bool

class NAVPVT:

    def __init__(self):
        rospy.init_node("hdg_hackers")
        self.pub_hdg = rospy.Publisher("/heading_hack", Bool, queue_size=1)
        self.pub_pos = rospy.Publisher("/position_hack", Bool, queue_size=1)
        self.hdg = Bool()
        self.pos = Bool()

        # rospy.Subscriber("/ublox/navpvt", NavPVT, self.callback)
        # self.hacked_msg = NavPVT()
        # self.heading_hacked = False
        # self.position_hacked = False
        self.cnt = 0

    # def callback(self, msg):
    #     self.hacked_msg.iTOW = msg.iTOW
    #     self.hacked_msg.year = msg.year
    #     self.hacked_msg.month = msg.month
    #     self.hacked_msg.day = msg.day
    #     self.hacked_msg.hour = msg.hour
    #     self.hacked_msg.min = msg.min
    #     self.hacked_msg.sec = msg.sec
    #     self.hacked_msg.valid = msg.valid
    #     self.hacked_msg.tAcc = msg.tAcc
    #     self.hacked_msg.nano = msg.nano
    #     self.hacked_msg.fixType = msg.fixType
    #     self.hacked_msg.flags = msg.flags
    #     self.hacked_msg.flags2 = msg.flags2
    #     self.hacked_msg.numSV = msg.numSV

    #     if self.position_hacked:
    #         # self.hacked_msg.lon = msg.lon + int(0.0001/1e-7)
    #         # self.hacked_msg.lat = msg.lat + int(0.0001/1e-7)
    #         self.hacked_msg.lon = 0
    #         self.hacked_msg.lat = 0
    #     else:
    #         self.hacked_msg.lon = msg.lon
    #         self.hacked_msg.lat = msg.lat

    #     self.hacked_msg.height = msg.height
    #     self.hacked_msg.hMSL = msg.hMSL
    #     self.hacked_msg.hAcc = msg.hAcc
    #     self.hacked_msg.vAcc = msg.vAcc
    #     self.hacked_msg.velN = msg.velN
    #     self.hacked_msg.velE = msg.velE
    #     self.hacked_msg.velD = msg.velD
    #     self.hacked_msg.gSpeed = msg.gSpeed

    #     if self.heading_hacked:
    #         self.hacked_msg.heading = 0
    #     else:
    #         self.hacked_msg.heading = msg.heading
        
    #     self.hacked_msg.sAcc = msg.sAcc
    #     self.hacked_msg.headAcc = msg.headAcc
    #     self.hacked_msg.pDOP = msg.pDOP
    #     self.hacked_msg.reserved1 = msg.reserved1
    #     self.hacked_msg.headVeh = msg.headVeh
    #     self.hacked_msg.magDec = msg.magDec
    #     self.hacked_msg.magAcc = msg.magAcc

    def hack_heading_toggle(self):
        self.hdg.data = not self.hdg.data
    
    def hack_position_toggle(self):
        self.pos.data = not self.pos.data

    def input_thread(self):
        while not rospy.is_shutdown():
            try:
                _in = int(input(f"Current: hdg={self.hdg} pos={self.pos}\nPress 0 for hdg, 1 for pos: "))
                
                if _in == 0:
                    self.hack_heading_toggle()
                if _in == 1:
                    self.hack_position_toggle()

            except ValueError:
                print("not appropriate value")

    def main(self):
        rate = rospy.Rate(20)
        # 입력 스레드를 시작
        threading.Thread(target=self.input_thread, daemon=True).start()
        
        while not rospy.is_shutdown():
            self.pub_hdg.publish(self.hdg)
            self.pub_pos.publish(self.pos)
            rate.sleep()

if __name__ == "__main__":
    navpvt = NAVPVT()
    navpvt.main()

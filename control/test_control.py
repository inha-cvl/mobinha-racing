import rospy

import sys
import signal

from ros_handler import ROSHandler
from libs.apid import APID
from libs.purepursuit import PurePursuit, PurePursuit_test

def signal_handler(sig, frame):
    sys.exit(0)

class Control():
    def __init__(self):
        self.RH = ROSHandler()
        self.APID = APID(self.RH)
        # self.PP = PurePursuit(self.RH)
        self.PP = PurePursuit_test(self.RH)

    def execute(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            acc = self.APID.execute()
            steer = self.PP.execute()
            self.RH.publish(acc, steer)
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    control = Control()
    control.execute()

if __name__ == "__main__":
    main()
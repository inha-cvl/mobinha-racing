import rospy

import sys
import signal

from ros_handler import ROSHandler
from apid import APID

def signal_handler(sig, frame):
    sys.exit(0)

class Control():
    def __init__(self):
        self.RH = ROSHandler()
        self.APID = APID(self.RH)

    def execute(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            acc = self.APID.execute()
            #steer = self.PP.execute() -> Pure-pursuit
            self.RH.publish(acc)
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    control = Control()
    control.execute()

if __name__ == "__main__":
    main()
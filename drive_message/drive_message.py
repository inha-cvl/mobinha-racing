import rospy

import sys
import signal

from ros_handler import ROSHandler
# from ros_sm_handler import ROSHandler
from libs.map_config import * 

def signal_handler(sig, frame):
    sys.exit(0)

class DriveMessage():
    def __init__(self, map):
        base_lla = get_base_lla(map)
        self.RH = ROSHandler(map, base_lla)

    def execute(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.RH.publish()
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    map = sys.argv[1]
    drive_message = DriveMessage(map)
    drive_message.execute()

if __name__ == "__main__":
    main()
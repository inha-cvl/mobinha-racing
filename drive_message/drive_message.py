import rospy

import sys
import signal

from ros_handler import ROSHandler

def signal_handler(sig, frame):
    sys.exit(0)

class DriveMessage():
    def __init__(self):
        self.RH = ROSHandler()

    def execute(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.RH.publish()
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    drive_message = DriveMessage()
    drive_message.execute()

if __name__ == "__main__":
    main()
import rospy
import sys
import signal

from ros_handler import ROSHandler

def signal_handler(sig, frame):
    sys.exit(0)

class Perception():
    def __init__(self):
        self.RH = ROSHandler()
        self.set_values()

    def set_values(self):
        pass

    def execute(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.RH.publish(self.RH.radar_objects)
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    perception = Perception()
    perception.execute()

if __name__ == "__main__":
    main()
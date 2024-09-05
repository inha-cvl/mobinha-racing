import rospy
import sys
import signal

from ros_handler import ROSHandler
import perception_handler as ph

def signal_handler(sig, frame):
    sys.exit(0)

class Perception():
    def __init__(self):
        self.RH = ROSHandler()

    def execute(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            clustered_list = ph.cluster_points(self.RH.radar_objects)
            #filtered_list = ph.filtering_by_spd(clustered_list, self.RH.est_veh_spd)
            self.RH.publish(clustered_list)
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    perception = Perception()
    perception.execute()

if __name__ == "__main__":
    main()
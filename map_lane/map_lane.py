import rospy
import sys
import signal
from ros_handler import ROSHandler

from local_path_test import LocalPathTest
from hd_map.map import MAP
def signal_handler(sig, frame):
    sys.exit(0)

class MapLane():
    def __init__(self):
        self.RH = ROSHandler()
        self.map = None
        self.lpt_use = False

    def map_publish(self):
        lmap_viz, mlmap_viz = self.map.get_vizs()
        self.RH.publish_map_viz(lmap_viz, mlmap_viz)
    
    def map_initialize(self):
        if self.RH.map_name != None:
            self.map = MAP(self.RH.map_name)

    def execute(self):
        while self.map == None:
            self.map_initialize()
        self.map_publish()
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.lpt_use:
                lp_result = self.lpt.execute(self.RH.local_pos)
                if lp_result == None:
                    local_path, local_kappa = None, None 
                else:
                    [ local_path, local_kappa ] = lp_result
                local_velocity = self.acc.execute(self.RH.local_pos, local_path, local_kappa)
                self.RH.publish(local_path, local_kappa, local_velocity)
            else:
                pass
            rate.sleep()
                

def main():
    signal.signal(signal.SIGINT, signal_handler)
    map_lane = MapLane()
    map_lane.execute()

if __name__ == "__main__":
    main()
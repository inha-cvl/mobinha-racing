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
        self.lpt = None
        self.lpt_use = True
        self.max_vel = 50

    def map_publish(self):
        lmap_viz, mlmap_viz = self.map.get_vizs()
        self.RH.publish_map_viz(lmap_viz, mlmap_viz)
    
    def map_initialize(self):
        if self.RH.map_name != None:
            self.map = MAP(self.RH.map_name)
            self.lpt = LocalPathTest(self.RH, self.map)

    def execute(self):
        while self.map == None:
            self.map_initialize()
        self.map_publish()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.lpt is None:
                print("pass")
                pass
            if self.lpt_use:
                lp_result = self.lpt.execute(self.RH.local_pos)
                if lp_result == None:
                    local_path, local_kappa = None, None 
                else:
                    [ local_path, local_kappa ] = lp_result
                local_velocity = self.lpt.get_velocity(self.max_vel)
                # if not work, 
                # local_velocity = self.max_vel / 3.6
                self.RH.publish(local_path, local_kappa, local_velocity)
            rate.sleep()
                

def main():
    signal.signal(signal.SIGINT, signal_handler)
    map_lane = MapLane()
    map_lane.execute()

if __name__ == "__main__":
    main()
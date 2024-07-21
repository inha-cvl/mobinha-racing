import rospy
import sys
import signal
from ros_handler import ROSHandler
import time

from local_path_test import LocalPathTest
from hd_map.map import MAP
from get_lane_number import GetLaneNumber

def signal_handler(sig, frame):
    sys.exit(0)

class MapLane():
    def __init__(self):
        self.RH = ROSHandler()
        self.map = None
        self.lpt = None
        self.lpt_use = True
        self.max_vel = 30

    def map_publish(self):
        self.RH.publish_map_viz(self.map.lmap_viz, self.map.mlmap_viz)
    
    def map_initialize(self):
        if self.RH.map_name != None:
            self.map = MAP(self.RH.map_name)
            self.gln = GetLaneNumber(self.RH, self.map)
            if self.lpt_use:
                self.lpt = LocalPathTest(self.RH, self.map)

    def execute(self):
        while self.map == None:
            self.map_initialize()
        time.sleep(3)
        self.map_publish()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.lpt is None:
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
            # lane_number
            lane_data = self.gln.execute(self.RH.local_pos)
            if lane_data is None:
                curr_lane_num = None
            else:
                curr_lane_num = lane_data
            self.RH.publish_lane_data(curr_lane_num)
            rate.sleep()
                

def main():
    signal.signal(signal.SIGINT, signal_handler)
    map_lane = MapLane()
    map_lane.execute()

if __name__ == "__main__":
    main()
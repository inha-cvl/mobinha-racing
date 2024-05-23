import rospy
import pymap3d as pm

import sys
import signal

from ros_handler import ROSHandler
from local_path.local_path_test import LocalPathTest
from longitudinal.adaptive_cruise_control import AdaptiveCruiseControl
from hd_map.map import MAP

def signal_handler(sig, frame):
    sys.exit(0)

class Planning():
    def __init__(self, map):
        self.RH = ROSHandler()
        self.map = MAP(map)
        self.lpt = LocalPathTest(self.RH, self.map)
        self.acc = AdaptiveCruiseControl(self.RH)

    def calc_local_position(self):
        if self.RH.current_position_lat == 0:
            return None
        x, y, _ = pm.geodetic2enu(self.RH.current_position_lat, self.RH.current_position_long, 7, self.map.base_lla[0], self.map.base_lla[1], self.map.base_lla[2])
        return [x, y]

    def map_publish(self):
        lmap_viz, mlmap_viz = self.map.get_vizs()
        self.RH.publish_map(lmap_viz, mlmap_viz)

    def execute(self):
        rate = rospy.Rate(10)
        self.map_publish()
        while not rospy.is_shutdown():
            local_pos = self.calc_local_position()
            lp_result = self.lpt.execute(local_pos)
            if lp_result == None:
                local_path, local_kappa = None, None 
            else:
                [ local_path, local_kappa ] = lp_result
            local_velocity = self.acc.execute(local_pos, local_path, local_kappa)
            self.RH.publish(local_pos, local_path, local_kappa, local_velocity)
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    map = sys.argv[1]
    planning = Planning(map)
    planning.execute()

if __name__ == "__main__":
    main()
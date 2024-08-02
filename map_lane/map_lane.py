import rospy
import sys
import signal
from ros_handler import ROSHandler
import time

from local_path_test import LocalPathTest
from hd_map.map import MAP
from libs.lanelet_handler import LaneletHandler

def signal_handler(sig, frame):
    sys.exit(0)

class MapLane():
    def __init__(self):
        self.RH = ROSHandler()
        self.map = None
        self.lpt = None
        self.set_values()

    
    def set_values(self):
        self.lpt_use = False
        self.max_vel = 30
        self.stacked_refine_obstacles = []
        self.obstacle_timestamps = []
        self.remian_duration = 10

    def map_publish(self):
        self.RH.publish_map_viz(self.map.lmap_viz, self.map.mlmap_viz)
    
    def map_initialize(self):
        if self.RH.map_name != None:
            self.map = MAP(self.RH.map_name)
            self.llh = LaneletHandler(self.RH, self.map)
            if self.lpt_use:
                self.lpt = LocalPathTest(self.RH, self.map)
            
    def update_obstacles(self, new_obstacles):
        current_time = time.time()
        if len(new_obstacles) > 0:
            self.stacked_refine_obstacles.extend(new_obstacles)
            self.obstacle_timestamps.append(current_time)
            
        self.stacked_refine_obstacles = [
            obs for obs, timestamp in zip(self.stacked_refine_obstacles, self.obstacle_timestamps) 
            if current_time - timestamp < self.remian_duration
        ]
        self.obstacle_timestamps = [
            timestamp for timestamp in self.obstacle_timestamps 
            if current_time - timestamp < self.remian_duration
        ]

    def execute(self):
        while self.map == None:
            self.map_initialize()
        self.map_publish()

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.lpt is None:
                pass
            if self.lpt_use:
                lp_result = self.lpt.execute(self.RH.local_pose)
                if lp_result == None:
                    local_path, local_kappa = None, None 
                else:
                    [ local_path, local_kappa ] = lp_result
                local_velocity = self.lpt.get_velocity(self.max_vel)
                self.RH.publish(local_path, local_kappa, local_velocity)
            
            refine_obstacles = self.llh.refine_obstacles_heading(self.RH.local_pose, [self.RH.sim_obstacles, self.RH.cam_obstacles, self.RH.lid_obstacles])
            self.update_obstacles(refine_obstacles)
            self.RH.publish_refine_obstacles(self.stacked_refine_obstacles)    
            
            lane_data, lane_id = self.llh.get_lane_number(self.RH.local_pose)

            if lane_data is None and lane_id is None:
                curr_lane_num = None
                curr_lane_id = None
            else:
                curr_lane_num = lane_data
                curr_lane_id = lane_id
            self.RH.publish_lane_data(curr_lane_num, curr_lane_id)
            rate.sleep()
                

def main():
    signal.signal(signal.SIGINT, signal_handler)
    map_lane = MapLane()
    map_lane.execute()

if __name__ == "__main__":
    main()
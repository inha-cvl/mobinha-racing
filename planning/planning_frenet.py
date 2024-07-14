#!/usr/bin/env python3
import os
import sys
toppath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(toppath)

import frenet_ltpl.frenet_local_path
import rospy
import signal
import time
import numpy as np


import frenet_ltpl 
from ros_handler import ROSHandler
from longitudinal.get_max_velocity import GetMaxVelocity

def signal_handler(sig, frame):
    sys.exit(0)

class Planning():
    def __init__(self, global_path_name):
        self.RH = ROSHandler()
        self.setting_values(global_path_name)

    def setting_values(self, global_path_name):
        self.set_start = False
        self.load_global_path(global_path_name)
        self.gmv = GetMaxVelocity(self.RH, global_path_name)
        self.flp = frenet_ltpl.frenet_local_path.FrenetLocalPath()

        self.current_index = -1
        self.subset_length = 4000
        self.current_s = 0
        self.current_d = 0
        self.current_d_d = 0
        self.current_d_dd = 0
    
    

    def load_global_path(self, global_path_name):
        toppath = os.path.dirname(os.path.realpath(__file__))
        csv_path = f'{toppath}/inputs/paths/{global_path_name}.csv'
        pkl_path = f'{toppath}/inputs/pkls/{global_path_name}.pkl'
        self.global_path = frenet_ltpl.global_path.GlobalPath(csv_path, pkl_path)
        self.ref_line = np.vstack((self.global_path.x, self.global_path.y)).T

    def set_start_pos(self):
        self.current_index = frenet_ltpl.get_s_coord.closest_path_index(self.ref_line, self.RH.local_pos, self.current_index)
        if self.current_index >= 0:
            self.set_start = True
            self.update_sub_path()
            print("Set Start Position")

    def update_sub_path(self):
        self.sub_start_idx = self.current_index # Ensure start_idx is not less than 0
        self.sub_end_idx = min(self.sub_start_idx + self.subset_length, len(self.global_path.x))
        self.sub_x = self.global_path.x[self.sub_start_idx:self.sub_end_idx]
        self.sub_y = self.global_path.y[self.sub_start_idx:self.sub_end_idx]
        self.sub_k = self.global_path.k[self.sub_start_idx:self.sub_end_idx]
        self.sub_w_right = self.global_path.w_right[self.sub_start_idx:self.sub_end_idx]
        self.sub_w_left = self.global_path.w_left[self.sub_start_idx:self.sub_end_idx]
        self.sub_max_speed = self.global_path.max_speed[self.sub_start_idx:self.sub_end_idx]
        self.sub_ref_line = np.vstack((self.sub_x, self.sub_y)).T
        _, _, _, self.local_kappa, self.csp = self.flp.generate_target_course(self.sub_x, self.sub_y, self.sub_k)

    
    
    def execute(self):
        rate = rospy.Rate(10)
        cnt = 0 
        while not rospy.is_shutdown():      
              
            if self.RH.local_pos is None:
                continue
        
            if not self.set_start:
                self.set_start_pos()
            else:
                self.current_index = frenet_ltpl.get_s_coord.closest_path_index(self.ref_line, self.RH.local_pos, 0)
                if self.current_index > len(self.global_path.x)-1:
                    self.RH.publish_frenet(local_path, self.local_kappa, 0)
                    return
                local_current_index = frenet_ltpl.get_s_coord.closest_path_index(self.sub_ref_line, self.RH.local_pos, 0)
                if  self.subset_length - 80 < (self.current_index % self.subset_length):
                    self.update_sub_path()
                    self.RH.publish_frenet(local_path, self.local_kappa, road_max_vel)
                    continue

                road_widths = {'w_right': self.sub_w_right[0], 'w_left': self.sub_w_left[0]}
                current_max_speed = self.sub_max_speed[0]

                local_path = self.flp.frenet_optimal_planning(
                    self.csp, local_current_index, self.RH.current_velocity, self.RH.current_long_accel,
                    self.current_d, self.current_d_d, self.current_d_dd, self.RH.object_list2, road_widths, current_max_speed)

                road_max_vel = self.gmv.get_max_velocity(self.RH.local_pos)
                self.RH.publish_frenet(local_path, self.local_kappa, road_max_vel)
            cnt += 1    
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    global_path_name = sys.argv[1]
    planning = Planning(global_path_name)
    time.sleep(0.5)
    planning.execute()

if __name__ == "__main__":
    main()
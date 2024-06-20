#!/usr/bin/env python3
import os
import sys
toppath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(toppath)
os.environ['OPENBLAS_NUM_THREADS'] = str(1)

import rospy
import signal
import datetime
import json
import configparser
import math
import graph_ltpl
import time

from ros_handler import ROSHandler
from longitudinal.adaptive_cruise_control import AdaptiveCruiseControl
from longitudinal.get_max_velocity import GetMaxVelocity

def signal_handler(sig, frame):
    sys.exit(0)

class Planning():
    def __init__(self):
        self.RH = ROSHandler()
        self.acc = AdaptiveCruiseControl(self.RH)
        self.gmv = None
        self.setting_values()

    def setting_values(self):
        toppath = os.path.dirname(os.path.realpath(__file__))
        track_param = configparser.ConfigParser()
        if not track_param.read(toppath + "/params/driving_task.ini"):
            raise ValueError('Specified online parameter config file does not exist or is empty!')

        track_specifier = json.loads(track_param.get('DRIVING_TASK', 'track'))
        globtraj_input_path =  toppath + "/inputs/traj_ltpl_cl/traj_ltpl_cl_" + track_specifier + ".csv"
        path_dict = {'globtraj_input_path':globtraj_input_path,
                    'graph_store_path': toppath + "/inputs/stored_graph.pckl",
                    'ltpl_offline_param_path': toppath + "/params/ltpl_config_offline.ini",
                    'ltpl_online_param_path': toppath + "/params/ltpl_config_online.ini",
                    'log_path': toppath + "/logs/graph_ltpl/",
                    'graph_log_id': datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S")
                    }

        self.gmv = GetMaxVelocity(self.RH, globtraj_input_path)
        self.ltpl_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=path_dict,visual_mode=False,log_to_file=False)
        self.ltpl_obj.graph_init()
        self.traj_set = {'left': None}
        self.set_start = False
        self.local_action_set = []
        print("LTPL Ready")

    def set_start_pos(self,local_pos):
        _, start_node = self.ltpl_obj.set_startpos(pos_est=local_pos, heading_est=math.radians(self.RH.
        current_heading)) 
        if start_node is not None:
            self.set_start = True
            print("Set Start Position")

    def execute(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():            
            if self.RH.local_pos is None:
                continue
        
            if not self.set_start:
                self.set_start_pos(self.RH.local_pos)
            else:
                for sel_action in ["right", "left", "straight", "follow"]: 
                    if sel_action in self.traj_set.keys():
                        break

                self.ltpl_obj.calc_paths(prev_action_id=sel_action, object_list=self.RH.object_list)
                local_action_set = []
                if self.traj_set[sel_action] is not None:
                    local_action_set = self.traj_set[sel_action][0][:, :]

                self.traj_set = self.ltpl_obj.calc_vel_profile(
                                                pos_est=self.RH.local_pos,
                                                vel_est=self.RH.current_velocity,
                                                vel_max=80/3.6,
                                                safety_d=60)[0]
                road_max_vel = self.gmv.get_max_velocity(self.RH.local_pos)
                self.RH.publish(local_action_set, road_max_vel)
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    planning = Planning()
    time.sleep(0.5)
    planning.execute()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import os
import sys
toppath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(toppath)
os.environ['OPENBLAS_NUM_THREADS'] = str(1)

import rospy
import threading
import signal
import datetime
import math
import graph_ltpl
import frenet_ltpl
import time
import copy
import numpy as np

from ros_handler import ROSHandler
from longitudinal.adaptive_cruise_control import AdaptiveCruiseControl
from longitudinal.get_max_velocity import GetMaxVelocity
from global_path.global_path_planner import GlobalPathPlanner

def signal_handler(sig, frame):
    sys.exit(0)

class Planning():
    def __init__(self):
        self.RH = ROSHandler()
        self.acc = AdaptiveCruiseControl(self.RH)
        self.gmv = None
        self.gpp = None
        self.setting_values()

    def setting_values(self):
        self.shutdown_event = threading.Event()

        self.specifiers = ['to_goal', 'race']

        to_goal_path_dict = self.get_path_dict(self.specifiers[0])
        self.to_goal_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=to_goal_path_dict,visual_mode=False,log_to_file=False)
        race_path_dict = self.get_path_dict(self.specifiers[1])
        self.to_goal_obj.graph_init()
        self.race_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=race_path_dict,visual_mode=False,log_to_file=False)
        self.race_obj.graph_init()
        self.ltpl_obj = None

        self.set_start = False
        self.start_move = False
        self.path_mode = 'tum'
        
        self.local_action_set = []
        self.prev_lap = 0

        self.flp = frenet_ltpl.frenet_local_path.FrenetLocalPath()
        self.gpp = GlobalPathPlanner(self.RH.map_name)
        self.pit_point = rospy.get_param("/pit_stop_zone_coordinate")
        rospy.loginfo('[Planning] Value set.')
    
    def get_path_dict(self,  specifier):
        toppath = os.path.dirname(os.path.realpath(__file__))
        globtraj_input_path =  toppath + "/inputs/traj_ltpl_cl/traj_ltpl_cl_" + specifier + ".csv"
        path_dict = {'globtraj_input_path':globtraj_input_path,
                    'graph_store_path': toppath + f"/inputs/stored_{specifier}_graph.pckl",
                    'ltpl_offline_param_path': toppath + "/params/ltpl_config_offline.ini",
                    'ltpl_online_param_path': toppath + "/params/ltpl_config_online.ini",
                    'log_path': toppath + "/logs/graph_ltpl/",
                    'graph_log_id': datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S")
                    }
        return path_dict

    def check_lap(self):
        if self.prev_lap != self.RH.lap_count:
            self.set_start = False
            return 'PASS'
        elif self.RH.lap_count > 5:
            return 'OVER'
        else:
            return 'YET'


    def set_start_pos(self):
        start_time = time.time()
        if self.RH.lap_count == 0:
            ltpl_obj = copy.deepcopy(self.to_goal_obj)
            track_specifier = self.specifiers[0]
        else:
            ltpl_obj = copy.deepcopy(self.race_obj)
            track_specifier = self.specifiers[1]
        
        _, start_node = ltpl_obj.set_startpos(pos_est=self.RH.local_pos, heading_est=math.radians(self.RH.
        current_heading))
        if start_node is not None:
            self.set_start = True
            self.start_move = True
            self.traj_set = {'left': None}
            self.ltpl_obj = ltpl_obj
            self.gmv = GetMaxVelocity(self.RH, track_specifier)
            rospy.loginfo(f'[Planning] Start position set. {time.time()-start_time}')
    
    def init_pit_values(self, pit_path):
        self.current_index = -1
        self.current_s = 0
        self.current_d = 0
        self.current_d_d = 0
        self.current_d_dd = 0

        self.pit_path_x = pit_path[0]
        self.pit_path_y = pit_path[1]
        self.pit_path_k = pit_path[2]
        self.pit_path_w_right = pit_path[3]
        self.pit_path_w_left = pit_path[4]
        self.pit_path_speed = pit_path[5]
        self.pit_path_viz = pit_path[6]

        self.pit_path_ref_line = np.vstack((self.pit_path_x, self.pit_path_y)).T
        _,_,_,self.pit_path_kappa, self.pit_path_csp = self.flp.generate_target_course(self.pit_path_x, self.pit_path_y, self.pit_path_k)


    def initd(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            check_lap = self.check_lap()
            if self.path_mode != 'tum':
                pass
            if check_lap == 'PASS':
                self.prev_lap = self.RH.lap_count
            while not self.set_start:            
                if self.RH.local_pos is not None:
                    self.set_start_pos()
                rate.sleep()
    
   

    def pitd(self):
        rate = rospy.Rate(20)
        pit_path = []
          
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            if self.RH.kiapi_signal == 'Pit_Stop' and self.path_mode != 'frenet':
                start_time = time.time()
                pit_path = self.gpp.get_shortest_path(self.RH.local_pos, self.pit_point)
                # x, y, k, w_right, w_left, max_speed
                if pit_path is not None:
                    self.init_pit_values(pit_path)
                    
                    self.path_mode = 'frenet'
                    rospy.loginfo(f'[Planning] Frenet global-path set. {time.time()-start_time}')
            elif self.path_mode == 'frenet':
                self.RH.publish_path(self.pit_path_viz)
                start_time = time.time()
                self.current_index = frenet_ltpl.get_s_coord.closest_path_index(self.pit_path_ref_line, self.RH.local_pos, 0)
                current_max_speed = self.pit_path_speed[self.current_index]
                local_path = self.flp.frenet_optimal_planning(
                    self.pit_path_csp, self.current_index, self.RH.current_velocity, self.RH.current_long_accel,
                    self.current_d, self.current_d_d, self.current_d_dd, self.RH.object_list2, [self.pit_path_w_left[self.current_index],self.pit_path_w_right[self.current_index]], current_max_speed)
                if local_path is not None:
                    rospy.loginfo(f'[Planning] Frenet local path created. {time.time()-start_time}')
                    road_max_vel = self.gmv.get_max_velocity(self.RH.local_pos)
                else:
                    rospy.logwarn(f'[Planning] Frenet local path is None. {time.time()-start_time}')
                    road_max_vel = 5
                self.RH.publish_frenet(local_path, self.pit_path_kappa, road_max_vel)
            rate.sleep()


    def executed(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            while self.start_move and self.path_mode == 'tum':
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
                                                vel_max=110/3.6,
                                                safety_d=60)[0]
                if self.RH.kiapi_signal == 'Pit_Stop':
                    road_max_vel = 7
                road_max_vel = self.gmv.get_max_velocity(self.RH.local_pos)
                self.RH.publish(local_action_set, road_max_vel)
                rate.sleep()
            rate.sleep()
            

def main():
    signal.signal(signal.SIGINT, signal_handler)
    planning = Planning()
    time.sleep(0.5)

    thread1 = threading.Thread(target=planning.initd)
    thread2 = threading.Thread(target=planning.executed)
    thread3 = threading.Thread(target=planning.pitd)

    thread1.start()
    thread2.start()
    thread3.start()


    try:
        thread1.join()
        thread2.join()
        thread3.join()

    except KeyboardInterrupt:
        planning.shutdown_event.set()
        thread1.join()
        thread2.join()
        thread3.join()
    
    rospy.loginfo("ROSManager has shut down gracefully.")

    planning.execute()

if __name__ == "__main__":
    main()
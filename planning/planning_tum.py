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
import time
import copy

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
        
        self.local_action_set = []
        self.prev_lap = 0

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
        
        _, start_node = ltpl_obj.set_startpos(pos_est=self.RH.local_pos, heading_est=math.radians(self.RH.current_heading))
        if start_node is not None:
            self.set_start = True
            self.start_move = True
            self.traj_set = {'left': None}
            self.ltpl_obj = ltpl_obj
            self.gmv = GetMaxVelocity(self.RH, track_specifier)
            rospy.loginfo(f'[Planning] Start position set. {time.time()-start_time}')

    def initd(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            check_lap = self.check_lap()
            if check_lap == 'PASS':
                print(self.prev_lap, self.RH.lap_count)
                self.prev_lap = self.RH.lap_count
                #print(self.prev_lap, self.RH.lap_count)
            while not self.set_start:            
                if self.RH.local_pos is not None:
                    self.set_start_pos()
                rate.sleep()


    def executed(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            while self.start_move:
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

    thread1.start()
    thread2.start()


    try:
        thread1.join()
        thread2.join()

    except KeyboardInterrupt:
        planning.shutdown_event.set()
        thread1.join()
        thread2.join()
    
    rospy.loginfo("ROSManager has shut down gracefully.")

    planning.execute()

if __name__ == "__main__":
    main()
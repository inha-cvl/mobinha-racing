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
import time
import copy


import graph_ltpl

from ros_handler import ROSHandler
from longitudinal.get_max_velocity import GetMaxVelocity
from global_path.global_path_planner import GlobalPathPlanner

def signal_handler(sig, frame):
    sys.exit(0)

class Planning():
    def __init__(self):
        self.RH = ROSHandler()
        self.gmv = None
        self.gpp = None
        self.setting_values()

    def setting_values(self):
        self.shutdown_event = threading.Event()

        self.specifiers = ['to_goal', 'race']
        self.race_mode = self.specifiers[0]

        start_time = time.time()
        to_goal_path_dict = self.get_path_dict(self.specifiers[0])
        self.to_goal_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=to_goal_path_dict,visual_mode=False,log_to_file=False)
        self.to_goal_obj.graph_init()
        race_path_dict = self.get_path_dict(self.specifiers[1])
        self.race_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=race_path_dict,visual_mode=False,log_to_file=False)
        self.race_obj.graph_init()
        self.ltpl_obj = None
        rospy.loginfo(f'[Planning] {self.specifiers} Global Path set took {round(time.time()-start_time, 4)} sec')

        self.start_pose_initialized = False
        self.first_initialized = False
        
        self.local_action_set = []
        self.prev_lap = rospy.get_param('/now_lap')
        self.pit_point = rospy.get_param("/pit_stop_zone_coordinate")

        self.gpp = GlobalPathPlanner(self.RH.map_name)
        
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

    def check_planning_state(self):
        planning_state = 'NONE'
        race_mode = self.race_mode

        if self.first_initialized == False:
            race_mode = 'to_goal'
            planning_state = 'INIT'
        elif self.prev_lap != self.RH.lap_count and self.RH.lap_count < 5 and self.race_mode != 'pit_stop': #If pass the goal point, 
            self.start_pose_initialized = False
            self.prev_lap = self.RH.lap_count
            race_mode = 'race'
            planning_state = 'INIT'
        elif (self.RH.lap_count >= 5 or self.RH.kiapi_signal == 5 )and self.race_mode != 'pit_stop':
            self.start_pose_initialized = False
            race_mode = 'pit_stop'
            planning_state = 'INIT'
        if self.start_pose_initialized == True:
            planning_state = 'MOVE'

        return planning_state, race_mode
        
    def set_start_pos(self, race_mode):
        start_time = time.time()
        if race_mode == 'to_goal':
            ltpl_obj = copy.deepcopy(self.to_goal_obj)
        elif race_mode == 'race':
            ltpl_obj = copy.deepcopy(self.race_obj)
        elif race_mode == 'pit_stop':
            ltpl_obj = copy.deepcopy(self.pit_stop_obj)
        
        _, start_node, g_path = ltpl_obj.set_startpos(pos_est=self.RH.local_pos, heading_est=math.radians(self.RH.
        current_heading))
        if start_node is not None:
            self.start_pose_initialized = True
            self.first_initialized = True
            self.traj_set = {'follow': None}
            self.ltpl_obj = ltpl_obj
            self.gmv = GetMaxVelocity(self.RH, race_mode)
            self.RH.publish_global_path(g_path)
            rospy.loginfo(f'[Planning] {race_mode} Start position set took {round(time.time()-start_time, 4)} sec')
    
    def planning_pit_stop(self):
        start_time = time.time()
        gpp_result = self.gpp.get_shortest_path(self.RH.local_pos, self.pit_point, 'pit_stop')
        if gpp_result:
            pit_stop_path_dict = self.get_path_dict('pit_stop')
            self.pit_stop_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=pit_stop_path_dict,visual_mode=False,log_to_file=False)
            self.pit_stop_obj.graph_init()
            self.start_pose_initialized = False
            rospy.loginfo(f'[Planning] pit_stop Global Path set took {round(time.time()-start_time, 4)} sec')

    def initd(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            planning_state, self.race_mode = self.check_planning_state()
            if planning_state == 'INIT':
                if self.race_mode == 'pit_stop':
                    self.planning_pit_stop()
                while not self.start_pose_initialized:            
                    if self.RH.local_pos is not None:
                        self.set_start_pos(self.race_mode)
                    rate.sleep()
            rate.sleep()

    def executed(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            while self.first_initialized:
                for sel_action in [ "right", "left", "straight", "follow"]: 
                    if sel_action in self.traj_set.keys():
                        break
                
                self.ltpl_obj.calc_paths(prev_action_id=sel_action, object_list=self.RH.object_list)
                
                local_action_set = []
                if self.traj_set[sel_action] is not None:
                    local_action_set = self.traj_set[sel_action][0][:, :]

                self.traj_set = self.ltpl_obj.calc_vel_profile(
                                                pos_est=self.RH.local_pos,
                                                vel_est=self.RH.current_velocity,
                                                vel_max=60/3.6,
                                                safety_d=40)[0]
                
                # Set Target Velocity
                if not self.RH.set_go:
                    road_max_vel = 0
                elif not self.start_pose_initialized:
                    road_max_vel = 5
                else:
                    max_vel = self.gmv.get_max_velocity(self.RH.local_pos)
                    if self.RH.lap_count == 0 : # 1 lap under 30 km/h 
                        road_max_vel = min(37/3.6, max_vel)
                    else:
                        road_max_vel = max_vel
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
    
    rospy.loginfo("[Planning] Has shut down gracefully.")

    planning.execute()

if __name__ == "__main__":
    main()
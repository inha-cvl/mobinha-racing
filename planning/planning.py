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
import planning_handler as ph

def signal_handler(sig, frame):
    os._exit(0)

class Planning():
    def __init__(self):
        self.RH = ROSHandler()
        self.gmv = None
        self.gpp = None
        self.setting_values(rospy.get_param("/now_lap"))

    def setting_values(self, now_lap):
        while self.RH.map_name is None:
            pass

        self.shutdown_event = threading.Event()
        self.gpp = GlobalPathPlanner(self.RH.map_name)

        #self.specifiers = ['to_goal_solchan', 'race_solchan']
        self.specifiers = ['to_goal_fast', 'race_fast']
        #self.specifiers = ['race_kcity', 'race_kcity']
        self.race_mode = self.specifiers[0]
        self.prev_race_mode = self.race_mode

        start_time = time.time()
        # to_goal_path_dict = self.get_path_dict(self.specifiers[0])
        # self.to_goal_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=to_goal_path_dict,visual_mode=False,log_to_file=False)
        # self.to_goal_obj.graph_init()

        gpp_result = self.gpp.get_shortest_path(self.RH.local_pos, rospy.get_param("/goal_coordinate2"), self.specifiers[0])
        if gpp_result:
            to_goal_path_dict = self.get_path_dict(self.specifiers[0])
            self.to_goal_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=to_goal_path_dict,visual_mode=False,log_to_file=False)
            self.to_goal_obj.graph_init()

        race_path_dict = self.get_path_dict(self.specifiers[1])
        self.race_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=race_path_dict,visual_mode=False,log_to_file=False)
        self.race_obj.graph_init()
        
        self.ltpl_obj = None
        self.global_path = None
        self.traj_set = {'straight': None}    
        rospy.loginfo(f'[Planning] {self.specifiers} Global Path set took {round(time.time()-start_time, 4)} sec')


        self.start_pose_initialized = False
        self.first_initialized = False
        self.prev_target_vel = 0
        self.first_lap = 0
        self.slow_mode = 'OFF'
        self.pit_stop_decel = 'OFF'
        

        self.local_action_set = []
        self.prev_lap = now_lap
        self.pit_point = rospy.get_param("/pit_stop_zone_coordinate")
        self.max_vel = float(rospy.get_param("/max_velocity"))/3.6
        self.bank_list = ['37','43','44','80','79','78','63','62','61','54','59','60',
                          '40','41','42','45','46','47','2','5','4','50','49','48',
                          '14', '13', '12','1','10','11']


        
        
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
            self.first_lap = self.RH.lap_count
        elif self.prev_lap != self.RH.lap_count and self.RH.lap_count < 10 and self.race_mode != 'pit_stop': #If pass the goal point, 
            self.start_pose_initialized = False
            self.prev_lap = self.RH.lap_count
            race_mode = 'race'
            planning_state = 'INIT'
        elif self.RH.kiapi_signal == 5 and self.race_mode != 'pit_stop':
            self.start_pose_initialized = False
            race_mode = 'pit_stop'
            planning_state = 'INIT'
        elif self.RH.kiapi_signal == 2 and self.race_mode != 'stop':
            race_mode = 'stop'
        elif self.RH.kiapi_signal == 3 and self.race_mode != 'slow_on':
            self.prev_race_mode = self.race_mode
            race_mode = 'slow_on'
        elif self.RH.kiapi_signal == 4 and self.race_mode != 'slow_off':
            race_mode = self.prev_race_mode
            self.slow_mode = 'OFF'

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
        
        _, start_node, g_path = ltpl_obj.set_startpos(pos_est=self.RH.local_pos, heading_est=math.radians(self.RH.current_heading))
        if start_node is not None:
            self.gmv = GetMaxVelocity(self.RH, race_mode)
            self.start_pose_initialized = True
            self.first_initialized = True
            self.ltpl_obj = ltpl_obj
            self.gpp.global_path = g_path
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
        
    def check_bank(self):
        if self.RH.current_lane_id in self.bank_list:
            return True
        else:
            return False
    
    def get_stop_distance(self, decel_factor=7.5):
        react_distance = self.RH.current_velocity * 2
        brake_distance = (self.RH.current_velocity)**2/(2*decel_factor)
        return react_distance + brake_distance

    def calculate_road_max_vel(
        self, 
        local_action_set,
        stop_vel_decrement=0.1,               # 기본값 0.1
        slow_vel=10/3.6,                      # 기본값 10/3.6 (약 2.78 m/s)
        slow_mode_threshold=0.1,              # 기본값 0.1
        interval_divisor_base=4.6,              # 기본값 5
        interval_factor=8.6                   # 기본값 9.6                   
    ):
        # 기본 조건: set_go가 False일 경우
        if not self.RH.set_go:
            return 0
        # 'stop' 모드 처리
        if self.race_mode == 'stop':
            if not self.check_bank():
                return -1
            if len(local_action_set) > 3:
                return local_action_set[2][5]
            return self.prev_target_vel - stop_vel_decrement

        # 'slow_on' 모드 처리
        elif self.race_mode == 'slow_on':
            if not self.check_bank():
                road_max_vel = slow_vel
                if self.RH.current_velocity <= road_max_vel + slow_mode_threshold:
                    self.slow_mode = 'ON'
                return road_max_vel
            if self.slow_mode == 'ON':
                return slow_vel
            if len(local_action_set) > 3:
                return local_action_set[2][5]
            return self.prev_target_vel - stop_vel_decrement

        # 'pit_stop' 모드 처리
        elif self.race_mode == 'pit_stop':
            if self.pit_stop_decel == 'OFF' and self.get_stop_distance() > self.gpp.get_remain_distance(self.RH.local_pos):
                self.pit_stop_decel = 'ON'
            if self.pit_stop_decel == 'ON':
                interval = self.RH.current_velocity / (self.gpp.get_remain_distance(self.RH.local_pos)/ (interval_divisor_base + (self.RH.current_velocity / interval_factor)))
                return max(self.RH.current_velocity - interval, 0)
            else:
                pass
        # 일반적인 경우 처리
        if len(local_action_set) < 3:
            return max(self.prev_target_vel - stop_vel_decrement, 0)
        
        return local_action_set[2][5]

    def check_lane_deaprture(self, action_set, localpos):
        if action_set is not None and len(action_set) > 0:
            dist = ph.distance(action_set[0][1], action_set[0][2], localpos[0], localpos[1])
            if dist <= 5:
                return 'Normal'
            elif 5 < dist < 10:
                return 'Warning'
            else:
                return 'Danger'
    
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
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            while self.first_initialized:

                if self.RH.current_signal == 5:
                    rospy.loginfo("[Planning] Current signal is 5, resetting to initial state.")
                    self.setting_values(self.prev_lap)  # 초기화
                    self.RH.set_values()
                    break
                
                for sel_action in ["right", "left","straight", "follow"]: 
                    if sel_action in self.traj_set.keys():
                        break
                
                self.ltpl_obj.calc_paths(prev_action_id=sel_action, object_list=self.RH.object_list)
                
                local_action_set = []
                try:
                    if self.traj_set[sel_action] is not None:
                        local_action_set = self.traj_set[sel_action][0][:, :]
                    try:
                        self.traj_set = self.ltpl_obj.calc_vel_profile(pos_est=self.RH.local_pos,vel_est=self.RH.current_velocity,vel_max=self.max_vel,safety_d=20)[0]
                    except Exception as e:
                        local_action_set = []
                        rospy.logerr(f"[Planning] {e}")
                except KeyError as e:
                    rospy.logerr(f"[Planning] {e}")
                    
                road_max_vel = self.calculate_road_max_vel(local_action_set)
                

                if len(local_action_set) > 0 and len(local_action_set[:]) > 5:
                    # TUM version
                    target_velocity = self.gmv.smooth_velocity_plan(local_action_set[:][5], self.prev_target_vel,road_max_vel)[1]
                    # mobinha version
                    #R_list = ph.calculate_R_list2(local_action_set)
                    #target_velocity = self.gmv.smooth_velocity_plan2(local_action_set[:][5], self.prev_target_vel, road_max_vel, R_list)[1]
                else:
                    target_velocity = road_max_vel

                res = self.check_lane_deaprture(local_action_set, self.RH.local_pos)
                if res == 'Warning':
                    target_velocity = self.prev_target_vel - 0.5 if self.prev_target_vel - 0.5 >= 0 else 0
                elif res == 'Danger':
                    target_velocity = 0

                self.prev_target_vel = target_velocity
                self.RH.publish(local_action_set, target_velocity)

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
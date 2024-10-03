#!/usr/bin/env python3
import os
import sys
toppath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(toppath)

import rospy
import threading
import signal
import csv
import datetime
import math
import time
import copy
import json
import numpy as np


from ros_handler import ROSHandler
from longitudinal.get_max_velocity import GetMaxVelocity
from global_path.global_path_planner import GlobalPathPlanner
import planning_handler as ph

LOCAL_PATH_LENGTH=150

def signal_handler(sig, frame):
    os._exit(0)

class Planning():
    def __init__(self):
        self.RH = ROSHandler()
        self.gmv = None
        self.gpp = None
        self.setting_values(rospy.get_param("/now_lap"))

    
    def setting_values(self, now_lap):
        while self.RH.map_name is None or self.RH.local_pos is None:
            pass
        
        self.shutdown_event = threading.Event()
        self.gpp = GlobalPathPlanner(self.RH.map_name)
        

        self.specifiers = ['to_goal', 'race']
        self.race_mode = self.specifiers[0]
        self.prev_race_mode = self.race_mode
        self.avoid_on = True
        self.object_detected = False

        start_time = time.time()

        gpp_result = self.gpp.get_shortest_path((self.RH.local_pos[0], self.RH.local_pos[1]), [437.763, -342.904], self.specifiers[0]) #[427.079, -337.119]
        self.to_goal_path = self.get_ref_path(self.specifiers[0])
        self.race_path = self.get_ref_path(self.specifiers[1])

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
        self.bank_list = ['1', '10', '11', '12', '13', '14', '37', '40', '41', '42', '43', '44', '45', '46', '47', '54', '59', '60']
        self.lane3_list = ['1', '54']
    
    def get_ref_path(self, specifier):
        toppath = os.path.dirname(os.path.realpath(__file__))
        globtraj_input_path =  toppath + "/inputs/traj_ltpl_cl/traj_ltpl_cl_" + specifier + ".csv"
        ref_path = []
        with open(globtraj_input_path, mode='r') as file:
            csv_reader = csv.DictReader(file, delimiter=';')
            for row in csv_reader:
                float_row = [float(value) for value in row.values()]
                ref_path.append(float_row)      
        return ref_path
    
    
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
            if self.prev_race_mode in ['slow_on', 'slow_off', 'stop']:
                race_mode = self.prev_race_mode
            else:
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
            global_path = copy.deepcopy(self.to_goal_path)
        elif race_mode == 'race':
            global_path = copy.deepcopy(self.race_path)
        elif race_mode == 'pit_stop':
            global_path = copy.deepcopy(self.pit_stop_path)
        
        self.now_idx = 0
        idx = ph.find_closest_index(global_path, self.RH.local_pos)

        if idx is not None:
            self.gmv = GetMaxVelocity(self.RH, race_mode)
            self.start_pose_initialized = True
            self.first_initialized = True
            self.global_path = global_path
            self.now_idx = idx
            g_path = [(float(point[0]), float(point[1])) for point in global_path]
            self.RH.publish_global_path(g_path)
            self.gpp.global_path = g_path
            rospy.loginfo(f'[Planning] {race_mode} Start position set took {round(time.time()-start_time, 4)} sec')
    
    def planning_pit_stop(self):
        start_time = time.time()
        gpp_result = self.gpp.get_shortest_path(self.RH.local_pos, self.pit_point, 'pit_stop')
        if gpp_result:
            self.pit_stop_path = self.get_ref_path('pit_stop')
            self.start_pose_initialized = False
            rospy.loginfo(f'[Planning] pit_stop Global Path set took {round(time.time()-start_time, 4)} sec')
        
    def check_bank(self):
        if self.RH.current_lane_id in self.bank_list:
            return True
        else:
            return False
    
    def get_stop_distance(self, decel_factor=2.7):
        react_distance = self.RH.current_velocity * 2
        brake_distance = (self.RH.current_velocity)**2/(2*decel_factor)
        return react_distance + brake_distance

    def path_update(self, trim_global_path):
        if len(trim_global_path) < 5:
            return trim_global_path
        final_global_path = trim_global_path.copy()  # Make a copy of the global path to modify
        
        object_list = self.RH.object_list  # List of objects

        obj_radius_front = 30 + (self.RH.current_velocity / 5)  # Radius for obstacle avoidance (front)
        obj_radius_rear = 30 + (self.RH.current_velocity / 10)  # Radius for obstacle avoidance (rear)
        
        updated_path = []
        check_object = []
        front_object = []
        check_object_distances = []
        for obj in object_list:
            s, d = ph.object2frenet(trim_global_path, [float(obj['X']), float(obj['Y'])])
            if s > -2 :
                if -trim_global_path[int(s)][3] < d < trim_global_path[int(s)][2]:
                    obj['s'] = s
                    obj['d'] = d 
                    obj_dist = ph.distance(self.RH.local_pos[0], self.RH.local_pos[1], float(obj['X']), float(obj['Y']))
                    obj['dist'] = obj_dist
                    check_object.append(obj)
                    check_object_distances.append(obj_dist)
                    
                    if -1 < d < 1 :
                        front_object.append(obj)


        self.RH.publish_target_object(check_object, check_object_distances)

        front_object = sorted(front_object, key=lambda x: x['s'])


        self.object_detected = False
        if self.avoid_on:
            for point in trim_global_path:
                x, y = point[0], point[1]
                w_right, w_left = point[2], point[3]
                x_normvec, y_normvec = point[4], point[5]
                updated_point = point.copy()
                lat_accel = 9.23

                do_evasive = False
                for obj in front_object:
                    #d_evade = self.RH.current_velocity+(self.RH.current_velocity**2-obj['v']**2)/(2*lat_accel)
                    print(obj['v'],"<", int(self.RH.current_velocity*3.6))
                    if obj['v'] < self.RH.current_velocity:
                        do_evasive = True
                        obj_x, obj_y = float(obj['X']), float(obj['Y'])
                
                if do_evasive:
                    # Check the relative position of the object
                    if obj_y > y:
                        obj_radius = obj_radius_front
                    else:
                        obj_radius = obj_radius_rear
                    
                    if ph.distance(x, y, obj_x, obj_y) <= obj_radius:
                        self.object_detected = True
                        if w_left >= w_right:
                            points = np.arange(3.95, w_left, 1.2)
                        else:
                            points = np.arange(-3.95, -w_right, -1.2 )

                        # 생성된 점들
                        generated_points = [(x + (-1 * x_normvec) * i, y + (-1 * y_normvec) * i) for i in points]

                        # 가장 가까운 점은 첫 번째 점
                        closest_point = generated_points[0]
                        updated_point[0] = closest_point[0]
                        updated_point[1] = closest_point[1]
                    
                updated_path.append(updated_point)

            # Replace only the points in the path that need to be updated
            for i, point in enumerate(trim_global_path):
                for obj in object_list:
                    obj_x, obj_y = obj['X'], obj['Y']
                    if obj_y > point[1]:
                        obj_radius = obj_radius_front
                    else:
                        obj_radius = obj_radius_rear
                    
                    if ph.distance(point[0], point[1], obj['X'], obj['Y']) <= obj_radius:
                        final_global_path[i] = updated_path[i]

        return final_global_path
    
    def calculate_acc_vel(
        self,
        updated_path,
        interped_vel
    ):
        object_list = self.RH.object_list 
        acc_object_d_v = []
        for obj in object_list:
            s, d = ph.object2frenet(updated_path, [float(obj['X']), float(obj['Y'])])
            if -1 < d < 1:
                obj_dist = ph.distance(self.RH.local_pos[0], self.RH.local_pos[1], float(obj['X']), float(obj['Y']))
                acc_object_d_v.append([obj_dist, float(obj['v'])])
        min_s = 200
        obj_v = 200
        for s, v in acc_object_d_v:
            if min_s > s:
                min_s = s
                obj_v = v/3.6
        safety_distance = 40

        #print("s, v:", min_s, obj_v)

        
        if min_s < 15:
        # if min_s < 10:
            status = "danger"
            target_v_ACC = -1

        elif min_s < safety_distance:
        # if 10 < min_s < 40:
            status = "close"
            target_v_ACC = obj_v*3.6 * 0.8

        elif safety_distance < min_s < safety_distance*1.4:
        # elif 40 < min_s < 80:
            status = "middle"
            target_v_ACC = obj_v*3.6 * min_s / safety_distance

        elif safety_distance*1.4 < min_s:
        # elif 80 < min_s:
            status = "far or none"
            # target_v_ACC = 999
            target_v_ACC = interped_vel[2]
            
        else:
            print("zone_error")
        
        # print("ACC target v: ", target_v_ACC)
        # print("Status: ", status)
        # print()

        return target_v_ACC
    

    def calculate_road_max_vel(
        self, 
        acc_vel, 
        path_len,
        stop_vel_decrement=0.1,               # 기본값 0.1
        slow_vel=10/3.6,                      # 기본값 10/3.6 (약 2.78 m/s)
        slow_mode_threshold=0.1,              # 기본값 0.1
        interval_divisor_base=4.6,              # 기본값 5
        interval_factor=8.6                   # 기본값 9.6                   
    ):
        # 기본 조건: set_go가 False일 경우
        if not self.RH.set_go:
            return 0
        if path_len >= 3:
            action_velocity = acc_vel
            # if self.RH.current_lane_id in self.lane3_list:
            #     action_velocity = action_velocity + self.max_vel * 0.2
            #action_velocity = self.RH.get_mean_action(velocity_list[2:3])
        # 'stop' 모드 처리
        if self.race_mode == 'stop' :
            if not self.check_bank():
                return -1
            if path_len >= 2:
                return action_velocity
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
            if path_len >= 2:
                return action_velocity
            return self.prev_target_vel - stop_vel_decrement

        # 'pit_stop' 모드 처리
        elif self.race_mode == 'pit_stop':
            if self.gpp.get_remain_distance(self.RH.local_pos) < 700:
                remain_dist = ph.distance(self.RH.local_pos[0], self.RH.local_pos[1], self.pit_point[0], self.pit_point[1])
                if self.pit_stop_decel == 'OFF' and self.get_stop_distance() > remain_dist:
                    self.pit_stop_decel = 'ON'
                if self.pit_stop_decel == 'ON':
                    interval = self.RH.current_velocity / (remain_dist/ (interval_divisor_base + (self.RH.current_velocity / interval_factor)))
                    return max(self.RH.current_velocity - interval, 0)
                else:
                    pass

        # 일반적인 경우 처리
        if path_len < 2:
            return max(self.prev_target_vel - stop_vel_decrement, 0)
        
        #print("final vel: ", action_velocity)
        return action_velocity

    def check_lane_deaprture(self, local_path, localpos):
        if local_path is not None and len(local_path) > 0:
            dist = ph.distance(local_path[0][0], local_path[0][1], localpos[0], localpos[1])
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
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            while self.first_initialized:

                if self.RH.current_signal == 5:
                    rospy.loginfo("[Planning] Current signal is 5, resetting to initial state.")
                    self.setting_values(self.prev_lap)  # 초기화
                    self.RH.set_values()
                    break
                
                #local path trim
                trimmed_path, self.global_path = ph.trim_and_update_global_path(self.global_path,self.RH.local_pos,LOCAL_PATH_LENGTH)

                #path update for obstacle
                updated_path = self.path_update(trimmed_path) 

                #path spline
                interped_path,R_list, interped_vel = ph.interpolate_path(updated_path, min_length=int(LOCAL_PATH_LENGTH/2))
                
                #ACC
                acc_vel = self.calculate_acc_vel(updated_path, interped_vel) # 맹글어야댐

                road_max_vel = self.calculate_road_max_vel(acc_vel, len(interped_path))     

                target_velocity = self.gmv.smooth_velocity_plan2(interped_vel, self.prev_target_vel, road_max_vel, R_list)[1]

                if self.race_mode == 'pit_stop' and len(interped_path) < 7:
                    target_velocity = -1

                res = self.check_lane_deaprture(interped_path, self.RH.local_pos)
                if res == 'Warning':
                    target_velocity = self.prev_target_vel - 0.5 if self.prev_target_vel - 0.5 >= 0 else 0
                elif res == 'Danger':
                    target_velocity = 0

                self.prev_target_vel = target_velocity
                self.RH.publish2(interped_path, R_list, interped_vel, target_velocity, self.race_mode)

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
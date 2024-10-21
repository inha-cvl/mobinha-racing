#!/usr/bin/env python3
import os
import sys
toppath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(toppath)

import rospy
import threading
import signal
import time
from datetime import datetime, timedelta
import copy

from ros_handler import ROSHandler
from longitudinal.get_max_velocity import GetMaxVelocity
from global_path.global_path_planner import GlobalPathPlanner
import planning_handler as ph

LOCAL_PATH_LENGTH=130
REAL_MAX_SPEED=115

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
        
        self.race_mode = 'to_goal'
        self.prev_race_mode = self.race_mode
        
        self.to_goal_path = None
        self.pit_stop_path = None
        
        self.lane_change_state = 'straight'
        self.lc_state_list = []
        self.prev_lc_state_list = None
        self.lc_state_list_remain_cnt = 0
        self.lc_state_time = None
        self.prev_lane_number = self.RH.current_lane_number
        self.diffrent_lane_cnt = 0
        self.change_point_state = ['normal', 'straight']
        self.change_point_cnt = 0
        self.acc_cnt = 0
        self.acc_reset = False
        self.system_warning = False

        self.start_pose_initialized = False
        self.first_initialized = False
        self.prev_target_vel = 0
        self.first_lap = 0
        self.slow_mode = 'OFF'
        self.pit_stop_decel = 'OFF'

        self.local_action_set = []
        self.prev_lap = now_lap
        self.pit_points = [rospy.get_param("/pit_stop_zone1_coordinate"), rospy.get_param("/pit_stop_zone2_coordinate"),rospy.get_param("/pit_stop_zone3_coordinate")]
        self.pit_point = self.pit_points[0]
        self.selected_lane = rospy.get_param("/selected_lane")
        self.goal_points = [ rospy.get_param("/lane1_goal_coordinate"), rospy.get_param("/lane2_goal_coordinate"), rospy.get_param("/lane3_goal_coordinate")]
        self.goal_point = self.goal_points[2]
        
        self.max_vel = float(rospy.get_param("/max_velocity"))/3.6
        self.bank_list = rospy.get_param("/curve_list")
   
    def get_kst(self):
        utc_now = datetime.utcnow()
        kst_now = utc_now + timedelta(hours=9)
        return kst_now.strftime('%Y-%m-%d %H:%M:%S')
        
    def check_planning_state(self):
        planning_state = 'NONE'
        race_mode = self.race_mode

        if self.first_initialized == False:
            race_mode = 'to_goal'
            planning_state = 'INIT'
            self.first_lap = self.RH.lap_count
        elif self.prev_lap != self.RH.lap_count and self.race_mode != 'pit_stop': 
            self.start_pose_initialized = False
            if self.RH.lap_count % 2 == 0 and self.RH.lap_count != 0 and self.prev_lap != self.RH.lap_count:
                vel_offset = 5/3.6 if self.RH.lap_count <= 6 else 7/3.6
                self.max_vel = min(self.max_vel + vel_offset, REAL_MAX_SPEED/3.6)
            self.selected_lane = ph.get_selected_lane(self.max_vel, self.RH.current_lane_number)  
            self.prev_lap = self.RH.lap_count
            if self.prev_race_mode in ['slow_on', 'slow_off', 'stop']:
                race_mode = self.prev_race_mode
            else:
                race_mode = 'to_goal'
            planning_state = 'INIT'
            self.change_point_state = ['normal', 'straight']
            self.change_point_cnt = 0
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
        elif ph.has_different_lane_number(self.prev_lane_number, self.RH.current_lane_number) and self.race_mode != 'pit_stop':
            self.diffrent_lane_cnt += 1
            if self.diffrent_lane_cnt > 8 :#and self.lane_change_state not in ['straight', 'follow']:
                self.diffrent_lane_cnt = 0
                self.start_pose_initialized = False
                #TODO: Pre-2 off 
                self.selected_lane = ph.get_selected_lane(self.max_vel, self.RH.current_lane_number)
                if self.race_mode == 'slow_on' and self.selected_lane == 1:
                    self.selected_lane = 2
                self.prev_lane_number = self.RH.current_lane_number   
                self.lc_state_list_remain_cnt = 0
                self.acc_reset = False
                self.acc_cnt = 0
                self.prev_lc_state_list = None
            race_mode = self.race_mode
            planning_state = 'INIT'
            self.change_point_state =[ 'normal', 'straight']
            self.change_point_cnt = 0
        if self.start_pose_initialized == True:
            planning_state = 'MOVE'

       
        return planning_state, race_mode
        
    def set_start_pos(self, race_mode):
        if race_mode == 'pit_stop':
            global_path = copy.deepcopy(self.pit_stop_path)
        else:
            race_mode = 'to_goal'
            global_path = copy.deepcopy(self.to_goal_path)
        
        if global_path is not None:
            self.gmv = GetMaxVelocity(self.RH, race_mode)
            self.start_pose_initialized = True
            self.first_initialized = True
            self.global_path = global_path
            g_path = [(float(point[0]), float(point[1])) for point in global_path]
            self.RH.publish_global_path(g_path)
            self.gpp.global_path = g_path
    
    def set_pit_point(self):
        if self.RH.current_lane_number <= 2:
            self.pit_point = self.pit_points[0]
        else:
            self.pit_point = self.pit_points[2]

    def planning_pit_stop(self):
        start_time = time.time()
        self.set_pit_point()
        gpp_result, gp = self.gpp.get_shortest_path(self.RH.local_pos, self.pit_point, 'pit_stop')
        if gpp_result:
            self.pit_stop_path = gp
            self.start_pose_initialized = False
            rospy.loginfo(f'[{self.get_kst()}] pit_stop set {round(time.time()-start_time, 2)} sec')
    
    def planning_to_goal(self):
        start_time = time.time()
        point = self.goal_points[self.selected_lane-1]
        gpp_result, gp = self.gpp.get_shortest_path(self.RH.local_pos, point, 'to_goal')
        if gpp_result:
            self.to_goal_path = gp
            self.start_pose_initialized = False
            rospy.loginfo(f'[{self.get_kst()}] to_goal set {round(time.time()-start_time, 2)} sec')
        
    def check_bank(self):
        if self.RH.current_lane_id in self.bank_list:
            return True
        else:
            return False
    
    def path_update(self, trim_global_path):
        if len(trim_global_path) < 5:
            return trim_global_path
        
        final_global_path = trim_global_path.copy()  # Make a copy of the global path to modify
        
        object_list = self.RH.object_list  # List of objects
        
        check_object = []
        front_object = []
        right_object = []
        self.lane_change_state = 'straight'

        long_avoidance_gap = 35
        lat_avoidance_gap = 3.7 if self.check_bank() else 3.5
        target_d = 3 if self.check_bank() else 2.7

        for obj in object_list:
            s, d = ph.object2frenet(trim_global_path, [float(obj['X']), float(obj['Y'])])
            l_th, r_th = ph.get_lr_threshold(trim_global_path, s)  
            if  r_th < d <l_th and -50 < s:
                obj['s'] = s
                obj['d'] = d
                obj['ttc'] = ph.calc_ttc(obj['dist'], obj['v'], self.RH.current_velocity)
                check_object.append(obj)
                if -target_d < d < target_d and -6 < s :
                    front_object.append(obj)
                    if s < 100:
                        self.lane_change_state = 'follow'
                if -4.8 < d < -1.25 and -5 < s < 20:
                    right_object.append(obj)


        self.RH.publish_target_object(check_object)

        front_object = sorted(front_object, key=lambda x: x['s'])

        overtaking_required = False
        for obj in front_object:
            overtakng = ph.calc_overtaking_by_ttc(obj['dist'], obj['v'], self.RH.current_velocity)
            if overtakng:
                overtaking_required = True
                closest_obj_idx_on_path = ph.find_closest_index(trim_global_path, [obj['X'], obj['Y']])
                closest_info = trim_global_path[closest_obj_idx_on_path]
                self.lc_state_list = ph.get_lane_change_state(obj['d'], closest_info[3], closest_info[2])
                if self.lc_state_list is not None:
                    if self.lc_state_list_remain_cnt < 5:
                        if self.prev_lc_state_list is None or self.prev_lc_state_list[0] == self.lc_state_list[0]:
                            self.lc_state_list_remain_cnt += 1
                            self.prev_lc_state_list = self.lc_state_list
                        else:
                            self.lc_state_list = self.prev_lc_state_list
                break

    
        if self.acc_cnt >= 70 and self.race_mode != 'pit_stop' and not overtaking_required and self.lc_state_list is not None:
            overtaking_required = True
            self.acc_reset = True
        
        path_updated = False
        avoid_on = False
        if self.race_mode not in ['pit_stop', 'slow_on'] and overtaking_required and self.lc_state_list is not None:
            for i, lc_state in enumerate(self.lc_state_list):
                if not path_updated:
                    for j, point in enumerate(trim_global_path):
                        x, y = point[0], point[1]
                        r_width, l_width = point[2], point[3]
                        x_normvec, y_normvec = point[4], point[5]
                        for obj in front_object:
                            overtakng = ph.calc_overtaking_by_ttc(obj['dist'], obj['v'], self.RH.current_velocity)
                            if overtakng or self.acc_reset:
                                if self.RH.current_lane_id in ['28', '29', '30', '2', '5', '4', '38', '37', '36']:
                                    around_detected = ph.check_around2(obj, check_object, lc_state)
                                else:
                                    around_detected = ph.check_around(obj, check_object, lc_state)
                                bsd_detected = ph.check_bsd(self.RH.left_bsd_detect, self.RH.right_bsd_detect, lc_state)
                                lidar_bsd_detected = ph.check_bsd(self.RH.left_lidar_bsd_detect, self.RH.right_lidar_bsd_detect, lc_state)
                                lat_avoidance_overed, avoidance_gap = ph.check_avoidance_gap_over(lc_state, l_width, r_width, lat_avoidance_gap, obj['d'])
                                if not around_detected and not bsd_detected and not lidar_bsd_detected and not lat_avoidance_overed:
                                    path_updated = True
                                    obj_x, obj_y = float(obj['X']), float(obj['Y'])
                                    obj_radius = long_avoidance_gap - (obj['v'] / 5) 
                                    distance_to_obj = ph.distance(x, y, obj_x, obj_y)
                                    self.lane_change_state = lc_state
                                    if distance_to_obj <= obj_radius or avoid_on:
                                        avoid_on = True
                                        shift_value = avoidance_gap if lc_state == 'left' else -avoidance_gap
                                        generated_point = (x + (-1 * x_normvec) * shift_value, y + (-1 * y_normvec) * shift_value)
                                        final_global_path[j][0] = generated_point[0]
                                        final_global_path[j][1] = generated_point[1]
                                        

        elif self.race_mode == 'slow_on' and self.RH.current_lane_id in ['17', '14', '1', '25', '26', '56', '42']:
            bsd_detected = ph.check_bsd(self.RH.left_bsd_detect, self.RH.right_bsd_detect, 'right')
            lidar_bsd_detected = ph.check_bsd(self.RH.left_lidar_bsd_detect, self.RH.right_lidar_bsd_detect, 'right')
            if not bsd_detected and not lidar_bsd_detected and len(right_object) == 0:
                for i, point in enumerate(trim_global_path):
                    shift_value = -4
                    self.lane_change_state = 'right'
                    if i > 5:
                        generated_path = (point[0] + (-1 * point[4]) * shift_value, point[1] + (-1 * point[5]) * shift_value)
                        final_global_path[i][0] = generated_path[0]
                        final_global_path[i][1] = generated_path[1]
                            

        else: #if BSD detected in following / straight mode, we have to change lane & update global path 
            change_point_caution = self.gpp.get_change_point_caution(trim_global_path, self.RH.local_pos, self.RH.current_velocity)
            if change_point_caution is not None :
                change_caution, lc_state, change_idx = change_point_caution
                bsd_detected = ph.check_bsd(self.RH.left_bsd_detect, self.RH.right_bsd_detect, lc_state)
                lidar_bsd_detected = ph.check_bsd(self.RH.left_lidar_bsd_detect, self.RH.right_lidar_bsd_detect, lc_state)
                if bsd_detected and lidar_bsd_detected:
                    self.change_point_state = ['warning', lc_state]
           
        return final_global_path
    
    def calculate_acc_vel(
        self,
        updated_path,
        interped_vel,
        stop_vel_decrement=0.1               # 기본값 0.1
    ):
        if len(interped_vel) > 3:
            if self.lane_change_state == 'follow':
                object_list = self.RH.object_list 
                acc_object_d_v = []
                target_d = 3 if self.check_bank() else 2.7
                    
                for obj in object_list:
                    s, d = ph.object2frenet(updated_path, [float(obj['X']), float(obj['Y'])])
                    if s> 0 and -target_d < d < target_d:
                        acc_object_d_v.append([float(obj['dist']), float(obj['v'])])
                min_s = 200
                obj_v = 200

                for s, v in acc_object_d_v:
                    if min_s > s:
                        min_s = s
                        obj_v = v

                safety_distance = 30

                ttc = ph.calc_ttc(min_s, obj_v, self.RH.current_velocity)
                safety_distance = ((self.RH.current_velocity*3.6)*1.3)-30
                safety_distance = min(max(safety_distance, 20), 100)
                margin = safety_distance - min_s
                offset = 0.8
                target_v_ACC = obj_v*0.8 - margin*offset

                self.acc_cnt += 1
            else:
                target_v_ACC = interped_vel[2]
   
        else:
            target_v_ACC = max(self.prev_target_vel - stop_vel_decrement, -1)
        
        if self.change_point_state[0] == 'warning':
            target_v_ACC = max(self.prev_target_vel-(stop_vel_decrement*15), -1)
            bsd_detected = ph.check_bsd(self.RH.left_bsd_detect, self.RH.right_bsd_detect, self.change_point_state[1])
            lidar_bsd_detected = ph.check_bsd(self.RH.left_lidar_bsd_detect, self.RH.right_lidar_bsd_detect, self.change_point_state[1])
            if not bsd_detected and not lidar_bsd_detected:
                self.change_point_cnt += 1
                if self.change_point_cnt > 2:
                    self.change_point_state = ['normal', 'straight']
                    target_v_ACC = self.prev_target_vel
                    self.change_point_cnt = 0
                    

        return target_v_ACC

    def calculate_road_max_vel(
        self, 
        acc_vel, 
        slow_vel=9.8/3.6,                      # 기본값 10/3.6 (약 2.78 m/s)
        slow_mode_threshold=0.1,              # 기본값 0.1
        interval_divisor_base=4.6,              # 기본값 5
        interval_factor=8.6                   # 기본값 9.6                   
    ):
        # 기본 조건: set_go가 False일 경우
        if not self.RH.set_go:
            return -1
        # 'stop' 모드 처리
        if self.race_mode == 'stop' :
            if not self.check_bank():
                return -1
            

        # 'slow_on' 모드 처리
        elif self.race_mode == 'slow_on':
            if not self.check_bank():
                road_max_vel = slow_vel
                if self.RH.current_velocity <= road_max_vel + slow_mode_threshold:
                    self.slow_mode = 'ON'
                return road_max_vel
            if self.slow_mode == 'ON':
                return slow_vel
            

        # 'pit_stop' 모드 처리
        elif self.race_mode == 'pit_stop':
            self.max_vel = 60/3.6 if self.max_vel > 60/3.6 else self.max_vel
            if self.gpp.get_remain_distance(self.RH.local_pos) < LOCAL_PATH_LENGTH:
                remain_dist = ph.distance(self.RH.local_pos[0], self.RH.local_pos[1], self.pit_point[0], self.pit_point[1])
                if self.pit_stop_decel == 'OFF' and ph.get_stop_distance(self.RH.current_velocity) > remain_dist:
                    self.pit_stop_decel = 'ON'
                if self.pit_stop_decel == 'ON':
                    interval = self.RH.current_velocity / (remain_dist/ (interval_divisor_base + (self.RH.current_velocity / interval_factor)))
                    acc_vel = max(self.RH.current_velocity - interval, -1)
        
        if self.RH.system_health in [1,2] and not self.system_warning:
            acc_vel = min(slow_vel*3, acc_vel)
            self.system_warning = True

        elif self.RH.system_health == 0 and self.system_warning:
            self.system_warning = False
        
        if self.system_warning:
            acc_vel = min(slow_vel*3, acc_vel)
            
        return acc_vel

    
    def initd(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            planning_state, self.race_mode = self.check_planning_state()
            if planning_state == 'INIT':
                if self.race_mode == 'pit_stop':
                    self.planning_pit_stop()
                else:
                    self.planning_to_goal()
                while not self.start_pose_initialized:            
                    if self.RH.local_pos is not None:
                        self.set_start_pos(self.race_mode)
                    rate.sleep()
            rate.sleep()

    def executed(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            while self.first_initialized:

                if self.RH.current_signal == 5:
                    rospy.loginfo("[Planning] Current signal is 5, resetting to initial state.")
                    self.setting_values(self.prev_lap)  # 초기화
                    self.RH.set_values()
                    break
                
                trimmed_path, self.global_path = ph.trim_and_update_global_path(self.global_path,self.RH.local_pos,LOCAL_PATH_LENGTH)
                updated_path = self.path_update(trimmed_path) 
                interped_path, R_list, interped_vel = ph.interpolate_path(updated_path, min_length=int(LOCAL_PATH_LENGTH/2))
                
                acc_vel = self.calculate_acc_vel(updated_path, interped_vel)

                road_max_vel = self.calculate_road_max_vel(acc_vel)     
                                
                if self.RH.lap_count == 0: # TODO: 0lap limit velocity
                    limit_vel = 29/3.6  
                else:
                    limit_vel = self.max_vel
                target_velocity = min(limit_vel, road_max_vel)

                if self.race_mode == 'pit_stop' and len(interped_path) < 10:
                    target_velocity = -1
                
                self.prev_target_vel = target_velocity
                self.RH.publish2(interped_path, R_list, interped_vel, target_velocity, self.race_mode, self.lane_change_state)

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
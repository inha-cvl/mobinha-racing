#!/usr/bin/env python3
import os
import sys
toppath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(toppath)

import rospy
import threading
import signal
import csv
import time
import copy

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
        
        self.race_mode = 'to_goal'
        self.prev_race_mode = self.race_mode
        self.lane_change_state = False
        self.lc_state_list = []
        self.prev_lc_state_list = None
        self.lc_state_list_remain_cnt = 0
        self.lc_state_time = None
        self.prev_lane_number = self.RH.current_lane_number
        self.diffrent_lane_cnt = 0

        #kcity = self.gpp.get_shortest_path((self.RH.local_pos[0], self.RH.local_pos[1]), [239.553, 41.007], self.specifiers[0]) # : KCITY

        self.start_pose_initialized = False
        self.first_initialized = False
        self.prev_target_vel = 0
        self.first_lap = 0
        self.slow_mode = 'OFF'
        self.pit_stop_decel = 'OFF'

        self.local_action_set = []
        self.prev_lap = now_lap
        self.pit_point = rospy.get_param("/pit_stop_zone_coordinate")
        self.selected_lane = rospy.get_param("/selected_lane")
        self.goal_points = [ rospy.get_param("/lane1_goal_coordinate"), rospy.get_param("/lane2_goal_coordinate"), rospy.get_param("/lane3_goal_coordinate")]
        
        self.max_vel = float(rospy.get_param("/max_velocity"))/3.6
        self.bank_list = ['1', '2', '4','5', '10', '11', '12', '13', '14', '26', '28', '29', '30', '31', '32', '33', '34', '35', '42', '47', '48', '58', '59', '60']
    
    def check_planning_state(self):
        planning_state = 'NONE'
        race_mode = self.race_mode

        if self.first_initialized == False:
            race_mode = 'to_goal'
            planning_state = 'INIT'
            self.first_lap = self.RH.lap_count
        elif self.prev_lap != self.RH.lap_count and self.race_mode != 'pit_stop': 
            self.start_pose_initialized = False
            self.prev_lap = self.RH.lap_count
            if self.prev_race_mode in ['slow_on', 'slow_off', 'stop']:
                race_mode = self.prev_race_mode
            # elif self.RH.lap_count >= 3 :
            #     race_mode = 'pit_stop'
            else:
                race_mode = 'to_goal'
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
        elif ph.has_different_lane_number(self.prev_lane_number, self.RH.current_lane_number) and self.race_mode != 'pit_stop':
            self.diffrent_lane_cnt += 1
            if self.diffrent_lane_cnt > 10:
                self.diffrent_lane_cnt = 0
                self.start_pose_initialized = False
                self.prev_lane_number = self.RH.current_lane_number   
                self.lc_state_list_remain_cnt = 0
                self.prev_lc_state_list = None 
            race_mode = self.prev_race_mode
            planning_state = 'INIT'
            self.prev_race_mode = self.race_mode
        if self.start_pose_initialized == True:
            planning_state = 'MOVE'

       
        return planning_state, race_mode
        
    def set_start_pos(self, race_mode):
        start_time = time.time()
        if race_mode == 'to_goal':
            global_path = copy.deepcopy(self.to_goal_path)
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
        gpp_result, gp, gp_rviz = self.gpp.get_shortest_path(self.RH.local_pos, self.pit_point, 'pit_stop')
        if gpp_result:
            self.pit_stop_path = gp
            self.start_pose_initialized = False
            rospy.loginfo(f'[Planning] pit_stop Global Path set took {round(time.time()-start_time, 4)} sec')
    
    def planning_to_goal(self):
        start_time = time.time()
        point = self.goal_points[self.selected_lane-1]
        name = f'to_goal{self.selected_lane}'
        gpp_result, gp, gp_rviz = self.gpp.get_shortest_path(self.RH.local_pos, point, name)
        if gpp_result:
            self.to_goal_path = gp
            self.start_pose_initialized = False
            rospy.loginfo(f'[Planning] to_goal Global Path set took {round(time.time()-start_time, 4)} sec')
        
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
        self.lane_change_state = 'straight'

        long_avoidance_gap = 37
        lat_avoidance_gap = 3.5

        for obj in object_list:
            s, d = ph.object2frenet(trim_global_path, [float(obj['X']), float(obj['Y'])])
            l_th, r_th = ph.get_lr_threshold(trim_global_path, s)  
            if  r_th < d <l_th and s > -50:
                obj['s'] = s
                obj['d'] = d
                check_object.append(obj)
                if -2.5 < d < 2.5 and s > -10:
                    front_object.append(obj)
                    if s < 100:
                        self.lane_change_state = 'follow'

        self.RH.publish_target_object(check_object)
        
        if self.race_mode == 'pit_stop':
            return final_global_path
        
        front_object = sorted(front_object, key=lambda x: x['s'])

        overtaking_required = False
        for obj in front_object:
            overtakng = ph.calc_overtaking_by_ttc(obj['dist'], obj['v'], self.RH.current_velocity)
            
            if overtakng:
                overtaking_required = True
                closest_obj_idx_on_path = ph.find_closest_index(trim_global_path, [obj['X'], obj['Y']])
                closest_info = trim_global_path[closest_obj_idx_on_path]
                if self.lc_state_list_remain_cnt < 5:
                    self.lc_state_list = ph.get_lane_change_state(obj['d'], closest_info[3], closest_info[2])
                    if self.lc_state_list is not None:
                        if self.prev_lc_state_list is None or self.prev_lc_state_list[0] == self.lc_state_list[0]:
                            self.lc_state_list_remain_cnt += 1
                            self.prev_lc_state_list = self.lc_state_list

                break
        
        
        path_updated = False
        lc_state_idx = 9
        if overtaking_required and self.lc_state_list is not None:
            for i, lc_state in enumerate(self.lc_state_list):
                if not path_updated:
                    updated_path = []
                    for point in trim_global_path:
                        x, y = point[0], point[1]
                        r_width, l_width = point[2], point[3]
                        x_normvec, y_normvec = point[4], point[5]
                        updated_point = point.copy()
                        for obj in front_object:
                            overtakng = ph.calc_overtaking_by_ttc(obj['dist'], obj['v'], self.RH.current_velocity)
                            if overtakng:
                                around_detected = ph.check_around(obj, check_object, lc_state)
                                bsd_detected = ph.check_bsd(self.RH.left_bsd_detect, self.RH.right_bsd_detect, lc_state)
                                lat_avoidance_overed, avoidance_gap = ph.check_avoidance_gap_over(lc_state, l_width, r_width, lat_avoidance_gap, obj['d'])
                                if not around_detected and not bsd_detected and not lat_avoidance_overed:
                                    path_updated = True
                                    lc_state_idx = i
                                    obj_x, obj_y = float(obj['X']), float(obj['Y'])
                                    obj_radius = long_avoidance_gap + (obj['v'] / 5)  # 앞쪽 반경
                                    distance_to_obj = ph.distance(x, y, obj_x, obj_y)
                                    if distance_to_obj <= obj_radius:
                                        shift_value = avoidance_gap if lc_state == 'left' else -avoidance_gap
                                        generated_point = (x + (-1 * x_normvec) * shift_value, y + (-1 * y_normvec) * shift_value)
                                        updated_point[0] = generated_point[0]
                                        updated_point[1] = generated_point[1]
                        updated_path.append(updated_point)


        if path_updated:
            self.lane_change_state = self.lc_state_list[lc_state_idx]
            for i, point in enumerate(trim_global_path):
                for obj in object_list:
                    obj_radius = long_avoidance_gap + (obj['v'] / 5)
                    if ph.distance(point[0], point[1], obj['X'], obj['Y']) <= obj_radius:
                        final_global_path[i] = updated_path[i]
            
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
                for obj in object_list:
                    s, d = ph.object2frenet(updated_path, [float(obj['X']), float(obj['Y'])])
                    if s> 0 and -2.5 < d < 2.5:
                        acc_object_d_v.append([float(obj['dist']), float(obj['v'])])
                min_s = 200
                obj_v = 200
                for s, v in acc_object_d_v:
                    if min_s > s:
                        min_s = s
                        obj_v = v

                safety_distance = 30

                ttc = ph.calc_ttc(min_s, obj_v, self.RH.current_velocity)
                safety_distance = 0.4*self.RH.current_velocity*3.6
                safety_distance = min(max(safety_distance, 20), 40)

                margin = safety_distance - min_s

                offset = 0.8
                target_v_ACC = obj_v - margin*offset
            else:
                target_v_ACC = interped_vel[2]
   
        else:
            target_v_ACC = max(self.prev_target_vel - stop_vel_decrement, -1)

        return target_v_ACC

    def calculate_road_max_vel(
        self, 
        acc_vel, 
        slow_vel=10/3.6,                      # 기본값 10/3.6 (약 2.78 m/s)
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
            else:
                return acc_vel

        # 'slow_on' 모드 처리
        elif self.race_mode == 'slow_on':
            if not self.check_bank():
                road_max_vel = slow_vel
                if self.RH.current_velocity <= road_max_vel + slow_mode_threshold:
                    self.slow_mode = 'ON'
                return road_max_vel
            if self.slow_mode == 'ON':
                return slow_vel
            else:
                return acc_vel

        # 'pit_stop' 모드 처리
        elif self.race_mode == 'pit_stop':
            self.max_vel = 60/3.6 if self.max_vel > 60/3.6 else self.max_vel
            if self.gpp.get_remain_distance(self.RH.local_pos) < LOCAL_PATH_LENGTH:
                remain_dist = ph.distance(self.RH.local_pos[0], self.RH.local_pos[1], self.pit_point[0], self.pit_point[1])
                if self.pit_stop_decel == 'OFF' and ph.get_stop_distance(self.RH.current_velocity) > remain_dist:
                    self.pit_stop_decel = 'ON'
                if self.pit_stop_decel == 'ON':
                    interval = self.RH.current_velocity / (remain_dist/ (interval_divisor_base + (self.RH.current_velocity / interval_factor)))
                    return max(self.RH.current_velocity - interval, -1)
                else:
                    pass

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
                
                #local path trim
                trimmed_path, self.global_path = ph.trim_and_update_global_path(self.global_path,self.RH.local_pos,LOCAL_PATH_LENGTH)

                #path update for obstacle
                updated_path = self.path_update(trimmed_path) 

                #path spline
                interped_path, R_list, interped_vel = ph.interpolate_path(updated_path, min_length=int(LOCAL_PATH_LENGTH/2))
                
                acc_vel = self.calculate_acc_vel(updated_path, interped_vel)
                road_max_vel = self.calculate_road_max_vel(acc_vel)     
                                
                if self.RH.lap_count == 100: # TODO: 0lap limit velocity
                    limit_vel = 28.5/3.6  #TODO: 0lap limit velocity
                else:
                    limit_vel = self.max_vel
                target_velocity = min(limit_vel, road_max_vel)

                if self.race_mode == 'pit_stop' and len(interped_path) < 7:
                    target_velocity = -1

                # res = ph.check_lane_deaprture(interped_path, self.RH.local_pos)
                # if res == 'Warning':
                #     target_velocity = max(0, self.prev_target_vel - 0.5)
                # elif res == 'Danger':
                #     target_velocity = 0

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
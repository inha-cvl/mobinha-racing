import numpy as np
import math
import configparser


MPS_TO_KPH = 3.6

class PurePursuit(object):
    def __init__(self, ros_handler):
        self.RH = ros_handler
        self.set_configs()
        self.prev_steer = 0

    def set_configs(self):
        config_file_path = './config.ini'
        config = configparser.ConfigParser()
        config.read(config_file_path)
        pp_config = config['PurePursuit']
        self.lfd_gain = float(pp_config['lfd_gain'])
        self.min_lfd = float(pp_config['min_lfd'])
        self.max_lfd = float(pp_config['max_lfd'])
        cm_config = config['Common']
        self.wheelbase = float(cm_config['wheelbase'])
        self.steer_ratio = float(cm_config['steer_ratio'])
        self.steer_max = float(cm_config['steer_max'])
        self.saturation_th = float(cm_config['saturation_th'])

    def execute(self):
        if len(self.RH.current_location) < 1 or self.RH.system_mode < 1:
            return 0
        

        # 1st lane, roll: 19~20
        # lfd = 15 # optimal at 30kph
        # lfd = 30 # optimal at 80kph
        # lfd = 36 # optimal at 100kph

        # 2nd lane, # roll: 5~6
        # lfd = 20 # optimal at 30kph
        lfd = 33 # 
        # lfd = 35 # optimal at 80kph  
        # lfd = 41 # optimal at 100kph

        # 3rd lane, # roll 0
        # lfd = 20 # optimal at 30kph
        # lfd = 24 # optimal at 50kph
        # lfd = 36 # optimal at 75pkh

        # lfd = self.lfd_gain * self.RH.current_velocity * MPS_TO_KPH
        # lfd = np.clip(lfd, self.min_lfd, self.max_lfd)
        
        vehicle_speed = self.RH.current_velocity * MPS_TO_KPH

        if self.RH.curved:
            if self.RH.lane_number == 1:
                lfd = 0.3*vehicle_speed + 6
                lfd = min(max(lfd, 15), 36)
            elif self.RH.lane_number == 2:
                lfd = 0.3*vehicle_speed + 11
                lfd = min(max(lfd, 20), 41)
            elif self.RH.lane_number == 3:
                lfd = 7/1125*vehicle_speed**2 - 67/225*vehicle_speed + 70/3
                lfd = min(max(lfd, 20), 36)
            else:
                lfd = self.lfd_gain*vehicle_speed
                lfd = min(max(lfd, self.min_lfd), self.max_lfd)
        else:
            lfd = self.lfd_gain*vehicle_speed
            lfd = min(max(lfd, self.min_lfd), self.max_lfd)
        print("original: ", lfd)

        # 1st lane
        # lfd = 28 # 70kph, 0.98
        lfd = 30 #80kph, 0.98
        lfd = 32 #90kph, 0.98
        lfd = 34 #90kph, 0.98

        # 2nd lane
        # lfd = 기존 # 60kph, 0.8
        # lfd = 33 #70kph, 0.85
        # lfd = 45 #80kph, 0.95
        # lfd = 50 #90kph, 0.98
        print("lfd:", lfd)



        point = self.RH.current_location
        route = self.RH.planned_route
        heading = math.radians(self.RH.current_heading)
        
        steering_angle = 0.
        for path_point in route:
            diff = path_point - point
            rotated_diff = diff.rotate(-heading)
            if rotated_diff.x > 0:
                dis = rotated_diff.distance()
                if dis >= lfd:
                    theta = rotated_diff.angle
                    steering_angle = np.arctan2(2*self.wheelbase*np.sin(theta), lfd*0.98)
                    self.RH.publish_lh(path_point)
                    break
        
        steering_angle = math.degrees(steering_angle)

        offset = min(max(self.RH.current_velocity * MPS_TO_KPH * 0.02 + 0.1, 1), 2.2)
        if self.RH.current_velocity * MPS_TO_KPH > 30:
            steering_angle = steering_angle * offset

        #saturated_angle = self.saturate_steering_angle(steering_angle)

        return steering_angle#-0.047

    def saturate_steering_angle(self, now):
        saturated_steering_angle = now
        diff = abs(self.prev_steer-now)
        if diff > self.saturation_th:
            if now>=0: 
                saturated_steering_angle = self.prev_steer+self.saturation_th
            else: 
                saturated_steering_angle = self.prev_steer-self.saturation_th
        self.prev_steer = saturated_steering_angle
        return saturated_steering_angle
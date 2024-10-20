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
                lfd = 0.2 * vehicle_speed + 14
                lfd = min(max(lfd, 28), 34)

                lfd_offset = 0.98

            elif self.RH.lane_number == 2:
                if vehicle_speed < 80:
                    lfd = 1.500e-03*vehicle_speed**3 -2.700e-01*vehicle_speed**2 + 1.635e+01*vehicle_speed -3.030e+02
                else:
                    lfd = 0.5*vehicle_speed + 5
                lfd = min(max(lfd, 25), 50)

                lfd_offset = -8.33333333e-07*vehicle_speed**4 + 2.26666667e-04*vehicle_speed**3 -2.26666667e-02*vehicle_speed**2 + 9.91833333e-01*vehicle_speed -1.52500000e+01
                lfd_offset = min(max(lfd_offset, 0.8), 0.98)

            elif self.RH.lane_number == 3:
                lfd = 0.6*vehicle_speed - 3
                lfd = min(max(lfd, 20), 45)

                lfd_offset = 0.005*vehicle_speed + 0.55 + 0.08 #0.04 :rainy
                lfd_offset = min(max(lfd_offset, 0.8), 0.85)

            else:
                lfd = self.lfd_gain*vehicle_speed
                lfd = min(max(lfd, self.min_lfd), self.max_lfd)
                lfd_offset = 0.95
        else:
            lfd = self.lfd_gain*vehicle_speed
            lfd = min(max(lfd, self.min_lfd), self.max_lfd)
            
            lfd_offset = 0.95

        ## 1st lane
        # 70kph
        # lfd = 28 
        # lfd_offset = 0.98

        # 80kph
        # lfd = 30 
        # lfd_offset = 0.98

        # 90kph
        # lfd = 32 
        # lfd_offset = 0.98
        
        # 100kph
        # lfd = 34 
        # lfd_offset = 0.98

        ## 2nd lane
        # 50kph 
        # lfd = 27
        # lfd_offset = 0.80

        # 60kph
        # lfd = 30
        # lfd_offset = 0.82

        # 70kph
        # lfd = 33 
        # lfd_offset = 0.85

        # 80kph
        # lfd = 45 
        # lfd_offset = 0.95

        # 90kph
        # lfd = 50 
        # lfd_offset = 0.98

        ## 3rd lane
        # 50kph
        # lfd = 27
        # lfd_offset = 0.8

        # 60kph
        # lfd = 33
        # lfd_offset = 0.85

        # 70kph
        # lfd = 39
        # lfd_offset = 0.85


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
                    steering_angle = np.arctan2(2*self.wheelbase*np.sin(theta), lfd*lfd_offset)
                    self.RH.publish_lh(path_point)
                    break
        
        steering_angle = math.degrees(steering_angle)

        steer_offset = min(max(self.RH.current_velocity * MPS_TO_KPH * 0.02 + 0.1, 1), 2.2)
        if self.RH.current_velocity * MPS_TO_KPH > 30:
            steering_angle = steering_angle * steer_offset

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
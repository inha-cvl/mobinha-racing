import numpy as np
import math
import configparser


MPS_TO_KPH = 3.6

class PurePursuit(object):
    def __init__(self, ros_handler):
        self.RH = ros_handler
        self.set_configs()
        self.saturation_th = 5 ## 미션1
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

    def execute(self):
        if len(self.RH.current_location) < 1 or self.RH.system_mode < 1:
            return 0
        
        lfd = self.lfd_gain * self.RH.current_velocity * MPS_TO_KPH
        lfd = np.clip(lfd, self.min_lfd, self.max_lfd)

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
                    steering_angle = np.arctan2(2*self.wheelbase*np.sin(theta), lfd)
                    self.RH.publish_lh(path_point)
                    break
        
        steering_angle_deg = math.degrees(steering_angle)
        if steering_angle_deg<0:
            final_steering_angle = max(-self.steer_max, steering_angle_deg)
        else:
            final_steering_angle = min(self.steer_max, steering_angle_deg)

        steering_angle = self.saturate_steering_angle(final_steering_angle)

        return steering_angle

    def saturate_steering_angle(self, now):
        saturated_steering_angle = now
        diff = abs(self.prev_steer-now)
        if diff > self.saturation_th:
            if now>=0: #left
                saturated_steering_angle = self.prev_steer+self.saturation_th
            else: #right
                saturated_steering_angle = self.prev_steer-self.saturation_th
        self.prev_steer = saturated_steering_angle
        return saturated_steering_angle


class PurePursuit_test(object):
    def __init__(self, ros_handler):
        self.RH = ros_handler
        self.set_configs()
        self.saturation_th = 5 ## 미션1
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

    def execute(self, path):
        if len(self.RH.current_location) < 1:
            return 0
        
        lfd = self.lfd_gain * self.RH.current_velocity * MPS_TO_KPH
        lfd = np.clip(lfd, self.min_lfd, self.max_lfd)

        point = self.RH.current_location
        # route = self.RH.planned_route
        route = path
        heading = math.radians(self.RH.current_heading)
        
        steering_angle = 0.
        for path_point in route:
            diff = path_point - point
            rotated_diff = diff.rotate(-heading)
            if rotated_diff.x > 0:
                dis = rotated_diff.distance()
                if dis >= lfd:
                    theta = rotated_diff.angle
                    steering_angle = np.arctan2(2*self.wheelbase*np.sin(theta), lfd)
                    self.RH.publish_lh(path_point)
                    break
        
        steering_angle_deg = math.degrees(steering_angle)
        if steering_angle_deg<0:
            final_steering_angle = max(-self.steer_max, steering_angle_deg)
        else:
            final_steering_angle = min(self.steer_max, steering_angle_deg)

        steering_angle = self.saturate_steering_angle(final_steering_angle)

        return steering_angle

    def saturate_steering_angle(self, now):
        saturated_steering_angle = now
        diff = abs(self.prev_steer-now)
        if diff > self.saturation_th:
            if now>=0: #left
                saturated_steering_angle = self.prev_steer+self.saturation_th
            else: #right
                saturated_steering_angle = self.prev_steer-self.saturation_th
        self.prev_steer = saturated_steering_angle
        return saturated_steering_angle
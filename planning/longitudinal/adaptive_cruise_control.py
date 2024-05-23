import numpy as np 
import configparser

MPS_TO_KPH = 3.6
KPH_TO_MPS = 1/3.6

class AdaptiveCruiseControl:
    def __init__(self, ros_handler):
        self.RH = ros_handler
        self.set_configs()

        self.object_dist= 0
        self.object_vel = 0
        self.dist_th = 5

        self.dangerous = 0

        self.velocity_list = [0., 10.0, 25., 40., 55, 80]
        self.max_accel_list = [5,4,3,2,2,2]
    
    def set_configs(self):
        config_file_path = './longitudinal/config.ini'
        config = configparser.ConfigParser()
        config.read(config_file_path)
        acc_config = config['AdaptiveCruiseControl']
        self.vehicle_length = float(acc_config['vehicle_length'])
        self.vel_gain = float(acc_config['velocity_gain'])
        self.dist_gain = float(acc_config['distance_gain'])
        self.time_gap = float(acc_config['time_gap'])
        self.max_velocity = float(acc_config['max_velocity'])

    def closest_index(self, path, point):
        path = np.array(path)
        point = np.array(point)
        distance = np.linalg.norm(path-point, axis=1)
        idx = np.argmin(distance)
        return idx

    def check_objects(self, local_pos, path):
        now = self.closest_index(path, local_pos)
        dist_to_goal = len(path)-now
        if dist_to_goal <= 0:
            self.object_dist = dist_to_goal-self.vehicle_length
            self.object_vel = 0

    def check_curvature_ratio(self, local_kappa):
        avg_curvature = abs(np.mean(local_kappa))*1000
        self.co = self.smoothed_deceleration(avg_curvature)
    
    def smoothed_deceleration(self, avg_curvature):
        if 5<avg_curvature <= 11:
            co = 0.35
        elif 2<avg_curvature<=5:
            co = 0.2
        elif 0.7<avg_curvature<=2:
            co = 0.1
        else:
            co = 0
        return co

    def get_dynamic_accel(self, ev):
        return np.interp(ev, self.velocity_list, self.max_accel_list)

    def get_target_velocity(self):
        ev = self.RH.current_velocity * MPS_TO_KPH
        tv = self.max_velocity

        # vel_error = ev - self.object_vel
        # safe_distance = ev*self.time_gap
        # dist_error = safe_distance-self.object_dist

        # acceleration = -(self.vel_gain*vel_error + self.dist_gain*dist_error)
        # tv = max(tv, tv-tv*(self.co))
        # alpha = self.get_dynamic_accel(ev)

        # print(ev, tv, vel_error, safe_distance, dist_error, acceleration)

        # out_vel = min(ev+acceleration+alpha, tv)

        acceleration = self.vel_gain * (tv - ev)
        out_vel = min(ev+acceleration, tv)

        return out_vel*KPH_TO_MPS

    def execute(self, local_pos, local_path, local_kappa):
        if local_path != None or local_kappa != None:
            self.check_objects(local_pos, local_path)
            self.check_curvature_ratio(local_kappa)
            vel = self.get_target_velocity()
        else:
            vel = 0
        return vel
    
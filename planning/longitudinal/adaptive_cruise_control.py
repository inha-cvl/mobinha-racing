import numpy as np 
import pymap3d as pm
import math
import scipy.interpolate
from scipy.spatial import KDTree
from planning.obstacle_utils import ObstacleUtils

class AdaptiveCruiseControl:
    def __init__(self,  vehicle_length, velocity_gain, distance_gain, time_gap, max_velocity, latitude, longitude, altitude):
        self.vel_gain = velocity_gain #bigger : sensitive with velocity
        self.dist_gain = distance_gain #smaller : sensitive with distance
        self.time_gap = time_gap
        self.vehicle_length = vehicle_length
        self.max_velocity = max_velocity

        self.base_lat = latitude
        self.base_lng = longitude
        self.base_alt = altitude

        self.object_dist= 0
        self.object_vel = 0
        self.dist_th = 9

        self.dangerous = 0
    
    def calc_error(self, path, vehicle_position):
        min_dist = 1000
        for point in path:
            dist = vehicle_position.distance(point)
            if dist < min_dist:
                min_dist = dist
        return min_dist
    
    def check_objects(self, path):
        goal = path[-1]
        distance_threshold = 0
        for point in path:
            distance_from_path = point.distance(goal)           
            if distance_from_path<=distance_threshold:
                self.object_dist = goal.distance(path[0])-(self.vehicle_length)
                self.object_vel = 0

    def calculate_curvature(self, local_kappa):
        avg_curvature = abs(np.mean(local_kappa))*1000
        co = self.smoothed_deceleration(avg_curvature)
        return avg_curvature, co
    
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

    def get_target_velocity(self, ego_vel, target_vel, co):
        vel_error = ego_vel - self.object_vel
        safe_distance = ego_vel*self.time_gap
        dist_error = safe_distance-self.object_dist

        acceleration = -(self.vel_gain*vel_error + self.dist_gain*dist_error)
        target_vel = max(self.max_velocity, target_vel-target_vel*(co))
        out_vel = min(ego_vel+acceleration+2, target_vel)

        if -dist_error < self.dist_th:
             out_vel -= 1.5
        return out_vel
    
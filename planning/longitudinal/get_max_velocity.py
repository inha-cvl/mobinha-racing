import csv
import numpy as np
import copy 

class GetMaxVelocity:
    def __init__(self, ros_handler, global_path_name):
        self.RH = ros_handler
        self.global_poses = []
        self.global_velocitys = []
        self.cut_dist = 30
        self.set_values(global_path_name)

    def set_values(self,global_path_name):
        csv_file = f'./inputs/traj_ltpl_cl/traj_ltpl_cl_{global_path_name}.csv'
        with open(csv_file, 'r', encoding='utf-8') as f:
            rdr = csv.reader(f, delimiter=';')
            for i, line in enumerate(rdr):
                if i < 2:
                    continue
                self.global_poses.append([float(line[0]),float(line[1])])
                self.global_velocitys.append(float(line[10]))

        self.speed_accel_map = np.array([
            [0.0, 1.5],
            [4.0/3.6, 1.5],
            [8.0/3.6, 1.5],
            [12.0/3.6, 1.5],
            [16.0/3.6, 1.5],
            [20.0/3.6, 1.5],
            [24.0/3.6, 1.5],
            [28.0/3.6, 1.5],
            [32.0/3.6, 1.5],
            [36.0/3.6, 1.5],
            [40.0/3.6, 1.425],
            [44.0/3.6, 1.325],
            [48.0/3.6, 1.2],
            [52.0/3.6, 1.1],
            [56.0/3.6, 1.025],
            [60.0/3.6, 0.975],
            [66.0/3.6, 0.825],
            [72.0/3.6, 0.625]
        ])

    def find_nearest_idx(self, local_pos):
        end_i = self.cut_dist if len(self.global_poses) > self.cut_dist else -1
        cut_global_poses = np.array(self.global_poses[0:end_i])
        dists = []
        for gp in cut_global_poses:
            ed = np.linalg.norm(gp-np.array(local_pos))
            dists.append(ed)
        dists = np.array(dists)
        return dists.argmin()
    
    def get_acceleration(self, current_velocity):
        # Interpolate to find the acceleration based on the current velocity
        if current_velocity <= self.speed_accel_map[0, 0]:
            return self.speed_accel_map[0, 1]
        elif current_velocity >= self.speed_accel_map[-1, 0]:
            return self.speed_accel_map[-1, 1]
        else:
            return np.interp(current_velocity, self.speed_accel_map[:, 0], self.speed_accel_map[:, 1])

    def cut_values(self, idx):
        self.global_poses = copy.deepcopy(self.global_poses[idx:])
        self.global_velocitys = copy.deepcopy(self.global_velocitys[idx:])
    
    def smooth_velocity_by_R(self, target_velocity, R_list):
        K = target_velocity * 15
        
        smoothed_velocities =  target_velocity - ( K / R_list[0] ) if target_velocity > 3 else target_velocity 

        return smoothed_velocities
    
    def smooth_velocity_plan(self, velocities, current_velocity, target_velocity, R_list, window_size=5):
        smoothed_velocities = np.copy(velocities)
        smoothed_velocities[0] = current_velocity

        for i in range(1, len(velocities)):
            delta_v = target_velocity - smoothed_velocities[i-1]
            sign = np.sign(delta_v)

            # Get adjusted acceleration based on current velocity
            adjusted_acceleration = self.get_acceleration(smoothed_velocities[i-1])
            delta_v = min(abs(delta_v), adjusted_acceleration) * sign
            smoothed_velocities[i] = smoothed_velocities[i-1] + delta_v

            # Check if we exceed the target velocity
            if (sign > 0 and smoothed_velocities[i] > target_velocity) or (sign < 0 and smoothed_velocities[i] < target_velocity):
                smoothed_velocities[i] = target_velocity

        # Adjust velocities based on curvature R
        K = target_velocity * 15
        for i in range(len(smoothed_velocities)):
            if R_list[i] != 0:  # Prevent division by zero
                adjusted_velocity = target_velocity - (K / R_list[i]) if target_velocity > 3 else target_velocity
                smoothed_velocities[i] = min(smoothed_velocities[i], adjusted_velocity)

        # # Apply moving average filter to smooth the velocities
        # for i in range(1, len(smoothed_velocities)):
        #     smoothed_velocities[i] = np.mean(smoothed_velocities[max(0, i-window_size):i+1])

        return smoothed_velocities


    
    def get_max_velocity(self, local_pos):
        min_idx = self.find_nearest_idx(local_pos)
        idx = min(min_idx + 3, len(self.global_poses))
        if idx >= len(self.global_velocitys):
            vel = 0
        else:
            vel = self.global_velocitys[idx]
        self.cut_values(min_idx)
        return vel
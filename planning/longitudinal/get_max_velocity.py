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

    def find_nearest_idx(self, local_pos):
        end_i = self.cut_dist if len(self.global_poses) > self.cut_dist else -1
        cut_global_poses = np.array(self.global_poses[0:end_i])
        dists = []
        for gp in cut_global_poses:
            ed = np.linalg.norm(gp-np.array(local_pos))
            dists.append(ed)
        dists = np.array(dists)
        return dists.argmin()

    def cut_values(self, idx):
        self.global_poses = copy.deepcopy(self.global_poses[idx:])
        self.global_velocitys = copy.deepcopy(self.global_velocitys[idx:])
    
    def smooth_velocity_plan(self, velocities, current_velocity, target_velocity, max_acceleration=2, window_size=5):
        smoothed_velocities = np.copy(velocities)
        smoothed_velocities[0] = current_velocity

        for i in range(1, len(velocities)):
            delta_v = target_velocity - smoothed_velocities[i-1]
            sign = np.sign(delta_v)
            delta_v = min(abs(delta_v), max_acceleration) * sign
            smoothed_velocities[i] = smoothed_velocities[i-1] + delta_v

        # Apply moving average filter to smooth the velocities
        for i in range(1, len(smoothed_velocities)):
            smoothed_velocities[i] = np.mean(smoothed_velocities[max(0, i-window_size):i+1])

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
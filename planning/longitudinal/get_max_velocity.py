import csv
import numpy as np
import copy 

class GetMaxVelocity:
    def __init__(self, ros_handler, global_csv):
        self.RH = ros_handler
        self.global_poses = []
        self.global_velocitys = []
        self.cut_dist = 30
        self.set_values(global_csv)

    def set_values(self, csv_file):
        with open(csv_file, 'r', encoding='utf-8') as f:
            rdr = csv.reader(f)
            for i, line in enumerate(rdr):
                if i < 2:
                    continue
                splited = line[0].split(';')
                self.global_poses.append([float(splited[0]),float(splited[1])])
                self.global_velocitys.append(float(splited[10]))

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
    
    def get_max_velocity(self, local_pos):
        min_idx = self.find_nearest_idx(local_pos)
        idx = min_idx + 3 if len(self.global_poses) > 4 else min_idx
        vel = self.global_velocitys[idx]
        print(vel)
        self.cut_values(min_idx)
        return vel
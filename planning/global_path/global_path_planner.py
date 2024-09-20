import os
import copy
import csv
import numpy as np

import global_path.libs.gp_utils as gput
import global_path

from scipy.interpolate import splprep, splev, interp1d


class GlobalPathPlanner():
    def __init__(self, map_name):
        self.set_maps(map_name)
        self.global_path = None
    
    def set_maps(self, map_name):
        map = global_path.libs.load_map.MAP(map_name)
        gput.lanelets = map.lanelets
        gput.tiles = map.tiles
        gput.tile_size = map.tile_size
        gput.graph = map.graph
        gput.lane_width = 3.25
    
    def to_csv(self, file_name, trajectory_info):
        top_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        file_path = f'{top_path}/inputs/traj_ltpl_cl/traj_ltpl_cl_{file_name}.csv'
        with open(file_path, 'w', newline='') as file:
            writer = csv.writer(file, delimiter=';')
            writer.writerow(['#x', 'y', 'w_right', 'w_left', 'x_normvec', 'y_normvec', 'alpha', 's', 'psj', 'kappa', 'vx', 'ax'])
            for tri in trajectory_info:
                writer.writerow(tri)
    
    def interpolate_path(self, path, sample_rate = 3, smoothing_factor = 30.0, interp_points=10):
        local_path = np.array([(point[0], point[1]) for point in path])

        sampled_indices = np.arange(0, len(local_path), sample_rate)
        sampled_local_path = local_path[sampled_indices]
        tck, u = splprep([sampled_local_path[:, 0], sampled_local_path[:, 1]], s=smoothing_factor)
        t_new = np.linspace(0, 1, len(sampled_local_path) * interp_points)
        path_interp = np.array(splev(t_new, tck)).T
        # 보간된 경로를 원래 경로 길이로 리샘플링
        original_length = len(local_path)
        resampled_indices = np.linspace(0, len(path_interp) - 1, original_length).astype(int)
        resampled_path = path_interp[resampled_indices]

        interp_path = resampled_path.tolist()
    
        return interp_path

    def get_shortest_path(self, start, goal, name): 
        start_ll = gput.lanelet_matching(start)
        goal_ll = gput.lanelet_matching(goal)

        s_node = gput.node_matching(start_ll)
        g_node = gput.node_matching(goal_ll)

        if s_node == g_node:
            shortest_path_id = ([s_node], 0)
        else:
            shortest_path_id = gput.dijkstra(s_node, g_node)

        if shortest_path_id is not None:
            shortest_path_id = shortest_path_id[0]
            final_path, final_ids, final_vs = gput.node_to_waypoints(shortest_path_id, start_ll, goal_ll)
            final_tr = []
            final_path = self.interpolate_path(final_path)
            copy_final_path = copy.deepcopy(final_path)
            copy_final_path.insert(0, final_path[0])
            copy_final_path.append(final_path[-1])

            vel_p, accel_p, dist_p = gput.adjust_velocity_profile(final_vs)
            s = 0

            for i,f in enumerate(final_path):
                before_after_pts = [copy_final_path[i], copy_final_path[i+2]]
                lw_left, lw_right = gput.get_lane_width(final_ids[i])
                A, B, theta = gput.calc_norm_vec(before_after_pts)
                Rk = gput.calc_kappa(f, before_after_pts)
                vx = vel_p[i]
                ax = accel_p[i]
                final_tr.append([f[0], f[1], lw_right, lw_left, A, B, 0, s, theta, Rk, vx, ax])
                s += 1
            
            path_viz = gput.PathViz(final_path, (255/255, 196/255, 18/255, 0.5))
            self.to_csv(name, final_tr)
            return True, path_viz
        else:
            return False, None

    def get_remain_distance(self, local_pose):
        if self.global_path is None:
            return 99999
        min_idx = gput.find_nearest_idx(self.global_path, local_pose)
        return (len(self.global_path)-min_idx)*4
    
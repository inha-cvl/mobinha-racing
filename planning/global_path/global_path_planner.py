import copy

import global_path.libs.gp_utils as gput
import global_path

class GlobalPathPlanner():
    def __init__(self, map_name):
        self.set_maps(map_name)
    
    def set_maps(self, map_name):
        map = global_path.libs.load_map.MAP(map_name)
        gput.lanelets = map.lanelets
        gput.tiles = map.tiles
        gput.tile_size = map.tile_size
        gput.graph = map.graph
        gput.lane_width = 3.25
    
    def get_shortest_path(self, start, goal): 
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
            final_path, final_ids, final_vs = gput.node_to_waypoints(shortest_path_id)

            # final_path, final_ids, final_vs = gput.cut_by_start_goal(start_ll, goal_ll, path_from_id)

            final_tr = []
            copy_final_path = copy.deepcopy(final_path)
            copy_final_path.insert(0, final_path[0])
            copy_final_path.append(final_path[-1])

            vel_p, accel_p, dist_p = gput.adjust_velocity_profile(final_vs)
            s = 0
            final_x = []
            final_y = []
            final_k = []
            final_lw_right = []
            final_lw_left = []
            final_max_speed = []


            for i,f in enumerate(final_path):
                before_after_pts = [copy_final_path[i], copy_final_path[i+2]]
                lw_left, lw_right = gput.get_lane_width(final_ids[i])
                A, B, theta = gput.calc_norm_vec(before_after_pts)
                Rk = gput.calc_kappa(f, before_after_pts)
                vx = vel_p[i]
                ax = accel_p[i]
                final_x.append(f[0])
                final_y.append(f[1])
                final_lw_right.append(lw_right)
                final_lw_left.append(lw_left)
                final_k.append(Rk)
                final_max_speed.append(vx)
                #final_tr.append([f[0], f[1], lw_right, lw_left, A, B, 0, s, theta, Rk, vx, ax])
                s += 1
            
            final_viz = gput.PathViz(final_path, (245/255, 212/255, 66/255, 0.5))
            return final_x, final_y, final_k, final_lw_right, final_lw_left, final_max_speed, final_viz
        else:
            return None

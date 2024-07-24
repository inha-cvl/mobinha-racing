import libs.gp_utils as gput
import math

class LaneletHandler:
    def __init__(self, ros_handler, map):
        self.RH = ros_handler
        self.MAP = map
        self.setting_values()

    def setting_values(self):
        gput.lanelets = self.MAP.lanelets
        gput.tiles = self.MAP.tiles
        gput.tile_size = self.MAP.tile_size
    
    def current_lane_number(self, local_pos):
        idnidx = gput.lanelet_matching(local_pos)
        if idnidx is not None:
            curr_lane_num = gput.lanelets[idnidx[0]]['laneNo']
            return curr_lane_num
        else:
            return -1

    def get_lane_number(self, local_pos):
        if local_pos is None:
            return None
        else:
            self.curr_lane_num = self.current_lane_number(local_pos)
            return self.curr_lane_num
    

    def refine_heading_by_lane(self, obs_pos):
        idnidx = gput.lanelet_matching(obs_pos)
        if idnidx is not None:
            waypoints = gput.lanelets[idnidx[0]]['waypoints']

            next_idx = idnidx[1]+3 if idnidx[1]+3 < len(waypoints)-4 else len(waypoints)-4
            
            prev_point = waypoints[idnidx[1]]
            next_point = waypoints[next_idx]

            delta_x = next_point[0] - prev_point[0]
            delta_y = next_point[1] - prev_point[1]
            
            heading = math.degrees(math.atan2(delta_y, delta_x))

            return heading
        else:
            return None
            

    def refine_obstacles_heading(self, obstacle_lists):
        refine_obstacles = []
        for obs_list in obstacle_lists:
            for obs in obs_list:
                refine_heading = self.refine_heading_by_lane([obs[1], obs[2]]) # insert x,y
                if refine_heading is not None:
                    obs.append(refine_heading)
                    refine_obstacles.append(obs)
                else:
                    continue
        return refine_obstacles
    

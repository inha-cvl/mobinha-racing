import libs.gp_utils as gput

class GetLaneNumber:
    def __init__(self, ros_handler, map):
        self.RH = ros_handler
        self.MAP = map
        self.setting_values()

    def setting_values(self):
        gput.lanelets = self.MAP.lanelets
        gput.tiles = self.MAP.tiles
        gput.tile_size = self.MAP.tile_size
    
    def current_lane_number(self, local_pos):
        l_id, _ = gput.lanelet_matching(local_pos)
        curr_lane_num = gput.lanelets[l_id]['laneNo']
        return curr_lane_num

    def execute(self, local_pos):
        if local_pos is None:
            return None
        else:
            self.curr_lane_num = self.current_lane_number(local_pos)
            return self.curr_lane_num

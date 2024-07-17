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
        idnidx = gput.lanelet_matching(local_pos)
        if idnidx is not None:
            curr_lane_num = gput.lanelets[idnidx[0]]['laneNo']
            return curr_lane_num
        else:
            return -1

    def execute(self, local_pos):
        if local_pos is None:
            return None
        else:
            self.curr_lane_num = self.current_lane_number(local_pos)
            return self.curr_lane_num

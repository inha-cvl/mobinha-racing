import json

class LaneletMap:
    def __init__(self, map_path):
        with open(map_path, 'r') as f:
            map_data = json.load(f)
        self.map_data = map_data
        self.lanelets = map_data['lanelets']
        self.groups = map_data['groups']
        self.precision = map_data['precision']
        self.for_viz = map_data['for_vis']
        self.basella = map_data['base_lla']
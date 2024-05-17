import rospy
import json
from hd_map.map_utils import *
from hd_map.lanelet import LaneletMap, TileMap
from hd_map.micro_lanelet_graph import MicroLaneletGraph


class MAP:
    def __init__(self, map):
        file_path = f'./hd_map/maps/{map}.json'
        with open(file_path, 'r') as file:
            data = json.load(file)

        self.base_lla = data['base_lla']

        self.tile_size = 5
        cut_dist = 15

        self.lmap = LaneletMap(file_path)
        tmap = TileMap(self.lmap.lanelets, self.tile_size)
        mlg = MicroLaneletGraph(self.lmap, cut_dist)
        
        self.graph = mlg.graph
        self.lanelets = mlg.lanelets
        self.tiles = tmap.tiles

    
    def get_vizs(self):
        lmap_viz = LaneletMapViz(self.lanelets, self.lmap.for_viz)
        mlmap_viz = MicroLaneletGraphViz(self.lanelets, self.graph)
        return lmap_viz, mlmap_viz
    
import json
import pickle
import os
from hd_map.libs.map_utils import *
from hd_map.libs.lanelet import LaneletMap, TileMap
from hd_map.libs.micro_lanelet_graph import MicroLaneletGraph

class MAP:
    def __init__(self, map):
        file_path = f'./hd_map/maps/{map}.json'
        pickle_file = f'./hd_map/pkls/{map}.pkl'
        self.tile_size = 5
        
        if os.path.exists(pickle_file):
            with open(pickle_file, 'rb') as file:
                data = pickle.load(file)
                self.base_lla = data['base_lla']
                self.lmap = data['lmap']
                self.graph = data['graph']
                self.lanelets = data['lanelets']
                self.tiles = data['tiles']
                self.lmap_viz = data['lmap_viz']
                self.mlmap_viz = data['mlmap_viz']
        else:
            with open(file_path, 'r') as file:
                data = json.load(file)

            self.base_lla = data['base_lla']
            cut_dist = 15

            self.lmap = LaneletMap(file_path)
            tmap = TileMap(self.lmap.lanelets, self.tile_size)
            mlg = MicroLaneletGraph(self.lmap, cut_dist)
        
            self.graph = mlg.graph
            self.lanelets = mlg.lanelets
            self.tiles = tmap.tiles

            self.lmap_viz, self.mlmap_viz = self.get_vizs()
            
            with open(pickle_file, 'wb') as file:
                pickle.dump({
                    'base_lla': self.base_lla,
                    'lmap': self.lmap,
                    'graph': self.graph,
                    'lanelets': self.lanelets,
                    'tiles': self.tiles,
                    'lmap_viz': self.lmap_viz,
                    'mlmap_viz': self.mlmap_viz
                }, file)
    
    
    def get_vizs(self):
        lmap_viz = LaneletMapViz(self.lanelets, self.lmap.for_viz)
        mlmap_viz = MicroLaneletGraphViz(self.lanelets, self.graph)
        return lmap_viz, mlmap_viz
    
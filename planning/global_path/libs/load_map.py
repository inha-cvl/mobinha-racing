import json
import pickle
import os

import global_path

class MAP:
    def __init__(self, map):
        toppath = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))
        file_path = f'{toppath}/map_lane/hd_map/maps/{map}.json'
        pickle_file = f'{os.path.dirname(os.path.dirname(os.path.realpath(__file__)))}/pkls/{map}.pkl'
        self.tile_size = 5
        
        if os.path.exists(pickle_file):
            with open(pickle_file, 'rb') as file:
                data = pickle.load(file)
                self.base_lla = data['base_lla']
                self.lmap = data['lmap']
                self.graph = data['graph']
                self.lanelets = data['lanelets']
                self.tiles = data['tiles']
        else:
            with open(file_path, 'r') as file:
                data = json.load(file)

            self.base_lla = data['base_lla']
            cut_dist = 50

            self.lmap = global_path.libs.lanelet.LaneletMap(file_path)
            tmap = global_path.libs.lanelet.TileMap(self.lmap.lanelets, self.tile_size)
            mlg = global_path.libs.micro_lanelet_graph.MicroLaneletGraph(self.lmap, cut_dist)
        
            self.graph = mlg.graph
            self.lanelets = mlg.lanelets
            self.tiles = tmap.tiles
            
            with open(pickle_file, 'wb') as file:
                pickle.dump({
                    'base_lla': self.base_lla,
                    'lmap': self.lmap,
                    'graph': self.graph,
                    'lanelets': self.lanelets,
                    'tiles': self.tiles
                }, file)
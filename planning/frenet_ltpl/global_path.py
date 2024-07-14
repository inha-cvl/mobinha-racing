import pandas as pd
import pickle
import os 

class GlobalPath:
    def __init__(self, csv_path, pkl_path):
        if os.path.exists(pkl_path):
            with open(pkl_path, 'rb') as file:
                data = pickle.load(file)
                self.x = data['x']
                self.y = data['y']
                self.k = data['k']
                self.w_right = data['w_right']
                self.w_left = data['w_left']
                self.max_speed = data['max_speed']
        else:
            global_path = pd.read_csv(csv_path)
            self.x = global_path['x'].to_numpy()
            self.y = global_path['y'].to_numpy()
            self.k = global_path['kappa'].to_numpy()
            self.w_right = global_path['w_right'].to_numpy()
            self.w_left = global_path['w_left'].to_numpy()
            self.max_speed = global_path['vx'].to_numpy()

            with open(pkl_path, 'wb') as file:
                pickle.dump({
                    'x': self.x,
                    'y': self.y,
                    'k': self.k,
                    'w_right': self.w_right,
                    'w_left': self.w_left,
                    'max_speed': self.max_speed
                }, file)
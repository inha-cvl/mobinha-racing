import sys
import os

os.environ['OPENBLAS_NUM_THREADS'] = str(1)

toppath = os.path.dirname(os.path.realpath(__file__))
sys.path.append(toppath)

import datetime
import json
import configparser
import graph_ltpl
import math

from drive_msgs.msg import *

class LTPL:
    def __init__(self, ros_handler):
        self.RH = ros_handler
        self.setting_values()

    def setting_values(self):
        track_param = configparser.ConfigParser()
        if not track_param.read(toppath + "/params/driving_task.ini"):
            raise ValueError('Specified online parameter config file does not exist or is empty!')

        track_specifier = json.loads(track_param.get('DRIVING_TASK', 'track'))

        path_dict = {'globtraj_input_path': toppath + "/inputs/traj_ltpl_cl/traj_ltpl_cl_" + track_specifier + ".csv",
                    'graph_store_path': toppath + "/inputs/stored_graph.pckl",
                    'ltpl_offline_param_path': toppath + "/params/ltpl_config_offline.ini",
                    'ltpl_online_param_path': toppath + "/params/ltpl_config_online.ini",
                    'log_path': toppath + "/logs/graph_ltpl/",
                    'graph_log_id': datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S")
                    }

        self.ltpl_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=path_dict,visual_mode=False,log_to_file=False)
        self.ltpl_obj.graph_init()
        self.refline = graph_ltpl.imp_global_traj.src.import_globtraj_csv.\
            import_globtraj_csv(import_path=path_dict['globtraj_input_path'])[0]
    
        self.traj_set = {'left': None}
        self.set_start = False
        self.local_action_set = []
        print("LTPL Ready")
    
    def set_start_pos(self, local_pos):
        self.ltpl_obj.set_startpos(pos_est=local_pos, heading_est=math.radians(self.RH.current_heading)) 
        self.set_start = True
        print("Set Start Position")

    def execute(self, local_pos):
        if local_pos == None:
            return None

        if not self.set_start:
            self.set_start_pos(local_pos)
        
        for sel_action in ["right", "left", "straight", "follow"]: 
            if sel_action in self.traj_set.keys():
                break
            
        self.ltpl_obj.calc_paths(prev_action_id=sel_action, object_list=[])
        
        if self.traj_set[sel_action] is not None:
            self.local_action_set = self.traj_set[sel_action][0][:, :]
        
        self.traj_set = self.ltpl_obj.calc_vel_profile(pos_est=local_pos,vel_est=self.RH.current_velocity,vel_max=80/3.6)[0]

        return self.local_action_set
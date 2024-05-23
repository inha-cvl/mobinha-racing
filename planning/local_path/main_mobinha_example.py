import sys
import os
import pymap3d as pm

os.environ['OPENBLAS_NUM_THREADS'] = str(1)

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import math
import datetime
import json
import time
import configparser
import graph_ltpl

from drive_msgs.msg import *

class LTPL:
    def __init__(self):
        rospy.init_node('LTPL', anonymous=False)
        self.base_lla = [35.64750540757964, 128.40264207604886, 7]
        
        toppath = os.path.dirname(os.path.realpath(__file__))
        sys.path.append(toppath)
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

        self.ltpl_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=path_dict,visual_mode=False,log_to_file=True)

        # calculate offline graph
        self.ltpl_obj.graph_init()

        # set start pose based on first point in provided reference-line
        self.refline = graph_ltpl.imp_global_traj.src.import_globtraj_csv.\
            import_globtraj_csv(import_path=path_dict['globtraj_input_path'])[0]

        # df = pd.DataFrame(self.refline)
        # df.to_csv(f'../GlobalPath/ref_lines/{track_specifier}.csv', index=False)

    def execute(self):
        print("Start LTPL")
        self.set_protocol()
        
        self.vehicle_state = None
        self.mode = 0
        self.obstacles = []
        if self.check_vehicle_state():
            print("Init Vehicle Position")
            pos_est = self.vehicle_state[0:2]
            heading_est = self.vehicle_state[2]
            self.ltpl_obj.set_startpos(pos_est=pos_est, heading_est=heading_est)
        
        traj_set = {'left': None}
        print("Start Planning")

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
        
            for sel_action in ["right", "left", "straight", "follow"]: 
                if sel_action in traj_set.keys():
                    break

            obj_list = self.obstacles
            self.ltpl_obj.calc_paths(prev_action_id=sel_action, object_list=[])#obj_list)
            
            if traj_set[sel_action] is not None:
                local_action_set = traj_set[sel_action][0][:, :]
                self.send_data(local_action_set)

            traj_set = self.ltpl_obj.calc_vel_profile(pos_est=self.vehicle_state[0:2],vel_est=self.vehicle_state[3],vel_max=25)[0]

            rate.sleep()

    def set_protocol(self):
        rospy.Subscriber('/VehicleState', VehicleState, self.pose_cb)        
        rospy.Subscriber('/SystemStatus', SystemStatus, self.mode_cb)
        self.local_action_set_pub = rospy.Publisher('/ltpl/local_action_set', PoseArray, queue_size=1)
        

    def pose_cb(self, msg):
        
        x, y = self.conver_to_enu(msg.position.x, msg.position.y)
        self.vehicle_state = [x, y, math.radians(msg.heading.data), msg.velocity.data] # enu x, enu y, radian heading, m/s velocity

    def mode_cb(self, msg):
        self.mode = msg.systemMode.data

    def track_box_cb(self, msg):
        obs = []
        for i, box in enumerate(msg.boxes):
            obs.append({'X': box.pose.position.x, 'Y': box.pose.position.y, 'theta': box.pose.orientation.z, 'type': 'physical', 'id': i, 'length': 5.0, 'v': box.value})
        self.obstacles = obs

    def conver_to_enu(self, lat, lng):
        x, y, _ = pm.geodetic2enu(lat, lng, 20, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        return x, y

    def check_vehicle_state(self):
        while self.vehicle_state == None or self.mode < 1:
            time.sleep(0.01)
        return True

    def send_data(self, local_action_set):
        posearray = PoseArray()
        for set in local_action_set:
            pose = Pose()
            pose.position.x = set[1] #ref_x
            pose.position.y = set[2] #ref_y
            pose.position.z = set[0] #s
            pose.orientation.x = set[3] #psj
            pose.orientation.y = set[4] #kappa
            pose.orientation.z = set[5] #vx
            pose.orientation.w = set[6] #ax
            posearray.poses.append(pose)
        
        self.local_action_set_pub.publish(posearray)
        

def main():
    ltpl = LTPL()
    ltpl.execute()

if __name__ == '__main__':
    main()


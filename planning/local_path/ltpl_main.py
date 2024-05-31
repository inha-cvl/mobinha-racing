import sys
import os
from pyproj import Proj, Transformer

os.environ['OPENBLAS_NUM_THREADS'] = str(1)

import rospy
from geometry_msgs.msg import Point
from drive_msgs.msg import *
from visualization_msgs.msg import Marker, MarkerArray

import math
import datetime
import json
import time
import configparser
import graph_ltpl

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
        self.traj_set = {'left': None}
        self.local_pos = None
        self.current_heading = None
        self.set_start = False
        
        self.set_protocol()

    def execute(self):        
        
        if not self.check_vehicle_state():
            return []
        else:
            if self.set_start == False:
                pos_est = self.local_pos
                heading_est = math.radians(self.current_heading)
                self.ltpl_obj.set_startpos(pos_est=pos_est, heading_est=heading_est)
                self.set_start = True
                
        for sel_action in ["right", "left", "straight", "follow"]: 
            if sel_action in self.traj_set.keys():
                break

        self.ltpl_obj.calc_paths(prev_action_id=sel_action, object_list=[])#obj_list)
        
        local_action_set = []
        if self.traj_set[sel_action] is not None:
            local_action_set = self.traj_set[sel_action][0][:, :]

        self.traj_set = self.ltpl_obj.calc_vel_profile(pos_est=self.local_pos,
                                        vel_est=self.current_velocity,vel_max=80/3.6)[0]
        return local_action_set


    def set_protocol(self):
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        self.navigation_data_pub = rospy.Publisher('/NavigationData', NavigationData, queue_size=1)
        self.path_viz_pub = rospy.Publisher('/planning/local_path', Marker, queue_size=1)

    def check_vehicle_state(self):
        while self.local_pos == None or self.system_mode != 1:
            time.sleep(0.01)
        return True
    
    def vehicle_state_cb(self, msg):
        self.current_velocity = msg.velocity.data
        self.current_heading = msg.heading.data
        self.current_position_lat = msg.position.x
        self.current_position_long = msg.position.y
        x, y = self.conver_to_enu(msg.position.x, msg.position.y)
        self.local_pos = [x,y]

    def system_status_cb(self, msg):
        self.system_mode = msg.systemMode.data 

    def conver_to_enu(self, lat, lng):
        proj_wgs84 = Proj(proj='latlong', datum='WGS84')

        # ENU 좌표계를 정의합니다. (lat0, lon0, h0)는 기준점의 좌표입니다.
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=self.base_lla[0], lon_0=self.base_lla[1], h_0=self.base_lla[2])

        # Transformer 객체를 사용하여 변환기를 생성합니다.
        transformer = Transformer.from_proj(proj_wgs84, proj_enu)

        # WGS84 좌표를 ENU 좌표로 변환합니다.
        e, n, u = transformer.transform(lng, lat, 7)
        return e, n

    def send_data(self, local_action_set):
        if len(local_action_set) > 0:
            self.navigation_data = NavigationData()
            self.navigation_data.currentLocation.x = self.local_pos[0]
            self.navigation_data.currentLocation.y = self.local_pos[1]
            self.navigation_data.plannedVelocity.data = local_action_set[1][5]
            paths = []
            for set in local_action_set:
                point = Point()
                point.x = set[1]
                point.y = set[2]
                paths.append([set[1], set[2]])
                self.navigation_data.plannedRoute.append(point)
                self.navigation_data.plannedKappa.append(set[4])
            self.navigation_data_pub.publish(self.navigation_data)
            self.publish_path_viz(paths)
        
    def publish_path_viz(self, paths):
        if paths == None:
            return
        path_viz = self.LocalPathViz(paths)
        self.path_viz_pub.publish(path_viz)
    
    def LocalPathViz(self, waypoints):
        color =  [241, 76, 152, 1]
        return self.Path(waypoints, 999, 0.2, 1.5, (color[0]/255,color[1]/255, color[2]/255, 0.5))

    def KappaPathViz(self, waypoints):
        return self.Path(waypoints, 999, 0.2, 1.5, (150/255,59/255, 255/255, 0.5))

    def Path(self, waypoints, id_, z, scale, color):
        marker = self.Line('path', int(id_), scale, color, len(waypoints))
        for pt in waypoints:
            marker.points.append(Point(x=pt[0], y=pt[1], z=z))
        return marker

    def Line(self, ns, id_, scale, color, len):
        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
        marker.ns = ns
        marker.id = id_
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        return marker

def main():
    ltpl = LTPL()

    rate = rospy.Rate(20)

    while True:
        
        local_action_set = ltpl.execute()
        ltpl.send_data(local_action_set)

        rate.sleep()

if __name__ == '__main__':
    main()


import rospy
import math

from drive_msgs.msg import *
from geometry_msgs.msg import Point
from pyproj import Proj, Transformer


class ROSHandler():
    def __init__(self):
        rospy.init_node('planning', anonymous=False)
        
        self.set_values()
        self.set_publisher_protocol()
        self.set_subscriber_protocol()

    def set_values(self):
        self.system_mode = 0
        self.current_velocity = 0
        self.current_heading = 0
        self.current_signal = 0
        self.current_position_lat = 0
        self.current_position_long = 0
        self.local_pos = [0,0]
        self.prev_start_pos = [0,0]
        self.object_list = [] 
        self.transformer = None
        

    def set_publisher_protocol(self):
        self.navigation_data_pub = rospy.Publisher('/NavigationData', NavigationData, queue_size=1)
        
    def set_subscriber_protocol(self):
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        rospy.Subscriber('/DetectionData', DetectionData, self.detection_data_cb)

    def system_status_cb(self, msg):
        base_lla = msg.baseLLA
        if self.transformer == None:
            proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
            proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=base_lla[0], lon_0=base_lla[1], h_0=base_lla[2])
            self.transformer = Transformer.from_proj(proj_wgs84, proj_enu)
        self.system_mode = msg.systemMode.data 
        self.current_signal = msg.systemSignal.data
    
    def vehicle_state_cb(self, msg):
        self.current_velocity = msg.velocity.data
        self.current_heading = msg.heading.data
        self.current_position_lat = msg.position.x
        self.current_position_long = msg.position.y
        if self.transformer == None:
            return
        x, y, _ = self.transformer.transform(self.current_position_long, self.current_position_lat, 7) 
        self.local_pos = [x,y]
    
    def detection_data_cb(self, msg):
        object_list = []
        for i, object in enumerate(msg.objects):
            object_list.append({'X': object.position.x, 'Y': object.position.y, 'theta': math.radians(object.heading.data), 'type': 'physical', 'id': i, 'length': 3.0, 'v': object.velocity.data})
        self.object_list = object_list

    def publish(self, local_action_set, road_max_vel):
        if len(local_action_set) > 0:
            self.navigation_data = NavigationData()
            self.navigation_data.currentLocation.x = self.local_pos[0]
            self.navigation_data.currentLocation.y = self.local_pos[1]
            self.navigation_data.plannedVelocity.data = road_max_vel#min(local_action_set[1][5], road_max_vel)
            for set in local_action_set:
                point = Point()
                point.x = set[1]
                point.y = set[2]
                self.navigation_data.plannedRoute.append(point)
                self.navigation_data.plannedKappa.append(set[4])
            self.navigation_data_pub.publish(self.navigation_data)
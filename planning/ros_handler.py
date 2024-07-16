import rospy
import math
import numpy as np

from drive_msgs.msg import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class ROSHandler():
    def __init__(self):
        rospy.init_node('planning', anonymous=False)
        
        self.set_values()
        self.set_publisher_protocol()
        self.set_subscriber_protocol()

    def set_values(self):
        self.system_mode = 0
        self.kiapi_signal = 0
        self.current_velocity = 0
        self.current_heading = 0
        self.current_signal = 0
        self.current_position_lat = 0
        self.current_position_long = 0
        self.local_pos = [0,0]
        self.prev_start_pos = [0,0]
        self.object_list = []
        self.object_list2 =  np.array([])
        self.transformer = None
        self.current_lat_accel = 0
        self.current_long_accel = 0
        self.lap_count = 0


    def set_publisher_protocol(self):
        self.navigation_data_pub = rospy.Publisher('/NavigationData', NavigationData, queue_size=1)
        self.path_pub = rospy.Publisher('/test_path', Marker, queue_size=1)
        
    def set_subscriber_protocol(self):
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        rospy.Subscriber('/DetectionData', DetectionData, self.detection_data_cb)
        rospy.Subscriber('/CANOutput', CANOutput, self.can_output_cb)

    def system_status_cb(self, msg):
        self.map_name = msg.mapName.data   
        self.system_mode = msg.systemMode.data 
        self.current_signal = msg.systemSignal.data
        self.lap_count = msg.lapCount.data
        self.kiapi_signal = msg.kiapiSignal.data
    
    def vehicle_state_cb(self, msg):
        self.current_velocity = msg.velocity.data
        self.current_heading = msg.heading.data
        self.current_position_lat = msg.position.x
        self.current_position_long = msg.position.y
        self.local_pos = (msg.enu.x, msg.enu.y)
    
    def can_output_cb(self, msg):
        self.current_long_accel = float(msg.Long_ACCEL.data)
        self.current_lat_accel = float(msg.LAT_ACCEL.data)
    
    def detection_data_cb(self, msg):
        object_list = []
        position_list = []
        for i, object in enumerate(msg.objects):
            object_list.append({'X': object.position.x, 'Y': object.position.y, 'theta': math.radians(object.heading.data), 'type': 'physical', 'id': i, 'length': 4.0, 'v': object.velocity.data})
            position_list.append([object.position.x, object.position.y])
        self.object_list = object_list
        self.object_list2 = np.array(position_list)

    def publish(self, local_action_set, road_max_vel):
        if local_action_set is not None and len(local_action_set) > 0:
            self.navigation_data = NavigationData()
            self.navigation_data.plannedVelocity.data = min(local_action_set[1][5], road_max_vel)
            for set in local_action_set:
                point = Point()
                point.x = set[1]
                point.y = set[2]
                self.navigation_data.plannedRoute.append(point)
                self.navigation_data.plannedKappa.append(set[4])
            self.navigation_data_pub.publish(self.navigation_data)
    
    def publish_frenet(self, path, kappa, velocity):
        self.navigation_data = NavigationData()
        if path != None:
            for i, x in enumerate(path.x):
                point = Point()
                point.x = x
                point.y = path.y[i]
                self.navigation_data.plannedRoute.append(point)
        if kappa != None:
            for rk in kappa:
                self.navigation_data.plannedKappa.append(rk)
        self.navigation_data.plannedVelocity.data = velocity
        self.navigation_data_pub.publish(self.navigation_data)
    
    def publish_path(self, path_viz):
        self.path_pub.publish(path_viz)
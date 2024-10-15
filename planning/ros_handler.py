import rospy
import math
import numpy as np

from drive_msgs.msg import *
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class ROSHandler():
    def __init__(self):
        rospy.init_node('planning', anonymous=False)
        
        self.set_values()
        self.set_publisher_protocol()
        self.set_subscriber_protocol()

    def set_values(self):
        self.base_lla = [0,0,0]
        self.system_mode = 0
        self.kiapi_signal = 0
        self.set_go = False
        self.current_velocity = 0
        self.current_heading = 0
        self.current_signal = 0
        self.current_position_lat = 0
        self.current_position_long = 0
        self.left_bsd_detect = 0
        self.right_bsd_detect = 0
        self.local_pos = None
        self.prev_start_pos = [0,0]
        self.object_list = []
        self.transformer = None
        self.current_lat_accel = 0
        self.current_long_accel = 0
        self.current_lane_id = None
        self.current_lane_number = 0
        self.lap_count = 0
        self.map_name = None


    def set_publisher_protocol(self):
        self.navigation_data_pub = rospy.Publisher('/NavigationData', NavigationData, queue_size=1)
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.target_object_pub = rospy.Publisher('/planning/target_object', DetectionData, queue_size=1)
        
    def set_subscriber_protocol(self):
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        rospy.Subscriber('/DetectionData', DetectionData, self.detection_data_cb)
        rospy.Subscriber('/LaneData', LaneData, self.lane_data_cb)
        rospy.Subscriber('/CCANOutput', CCANOutput, self.ccan_output_cb)

    def system_status_cb(self, msg):
        self.map_name = msg.mapName.data   
        self.system_mode = msg.systemMode.data 
        self.current_signal = msg.systemSignal.data
        self.lap_count = msg.lapCount.data
        self.kiapi_signal = msg.kiapiSignal.data
        #selff
        #0:None, 1:Go, 2:Stop, 3:Slow On, 4:Slow Off, 5:Pit Stop
        if not self.set_go and self.kiapi_signal == 1:
            self.set_go = True
        
    def lane_data_cb(self, msg: LaneData):
        self.current_lane_id = str(msg.currentLane.id.data)
        self.current_lane_number =  int(msg.currentLane.laneNumber.data)

    
    def vehicle_state_cb(self, msg):
        self.current_velocity = msg.velocity.data
        self.current_heading = msg.heading.data
        self.current_positdion_lat = msg.position.x
        self.current_position_long = msg.position.y
        self.local_pos = (msg.enu.x, msg.enu.y)
    
    def ccan_output_cb(self, msg: CCANOutput):
        self.left_bsd_detect = int(msg.CF_Lca_IndLeft.data) # 0 : none, 1 : detect
        self.right_bsd_detect = int(msg.CF_Lca_IndRight.data) 
    
    def detection_data_cb(self, msg):
        object_list = []
        position_list = []
        for i, object in enumerate(msg.objects):
            object_list.append({'X': object.position.x, 'Y': object.position.y, 'theta': math.radians(object.heading.data), 'type': 'physical', 'id': i, 'length': 4.0, 'v': object.velocity.data, 'dist': object.distance.data})
            position_list.append([object.position.x, object.position.y])
        self.object_list = object_list

    def publish(self, local_action_set, target_velocity, race_mode):
        
        if local_action_set is not None and len(local_action_set) > 0:
            self.navigation_data = NavigationData()
            self.navigation_data.targetVelocity.data = target_velocity
            self.navigation_data.raceMode.data = str(race_mode)
            for set in local_action_set:
                point = Point()
                point.x = set[1]
                point.y = set[2]
                self.navigation_data.plannedRoute.append(point)
                self.navigation_data.plannedKappa.append(set[4])
            self.navigation_data_pub.publish(self.navigation_data)
            
    
    def publish2(self, local_path, R_list, velocity_list, target_velocity, race_mode, planning_mode):
        if local_path is not None and len(local_path) > 0:
            self.navigation_data = NavigationData()
            self.navigation_data.targetVelocity.data = target_velocity
            self.navigation_data.raceMode.data = str(race_mode)
            self.navigation_data.planningMode.data = str(planning_mode)
            for i, set in enumerate(local_path):
                point = Point()
                point.x = set[0]
                point.y = set[1]
                self.navigation_data.plannedRoute.append(point)
                self.navigation_data.plannedKappa.append(R_list[i])
                self.navigation_data.plannedVelocity.append(velocity_list[i])
            self.navigation_data_pub.publish(self.navigation_data)
    
    
    def publish_target_object(self, object_list):
        detection_data = DetectionData()
        for i, obj in enumerate(object_list):
            object_info = ObjectInfo()
            object_info.type.data = 0
            object_info.position.x = obj['X']
            object_info.position.y = obj['Y']
            object_info.heading.data = math.degrees(obj['theta'])
            object_info.velocity.data = obj['ttc']
            object_info.distance.data = obj['d']
            detection_data.objects.append(object_info)
        
        self.target_object_pub.publish(detection_data)


    
    def publish_global_path(self, waypoints):
        path = Path()
        path.header.frame_id = "world" 
        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "gpath"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.global_path_pub.publish(path)
    
    def get_mean_action(self, local_action_set):
        action_mean = 0
        for set in local_action_set:
            action_mean += set[5]
        return action_mean / 5
            
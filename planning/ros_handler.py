import rospy

from drive_msgs.msg import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class ROSHandler():
    def __init__(self):
        rospy.init_node('Planning', anonymous=False)
        
        self.set_values()
        self.set_publisher_protocol()
        self.set_subscriber_protocol()

    def set_values(self):
        self.system_status = 0
        self.current_velocity = 0
        self.current_signal = 0
        self.current_position_lat = 0
        self.current_position_long = 0

    def set_publisher_protocol(self):
        self.navigation_data_pub = rospy.Publisher('/NavigationData', NavigationData, queue_size=1)
        self.lmap_viz_pub = rospy.Publisher('/lmap', MarkerArray, queue_size=1)
        self.mlmap_viz_pub = rospy.Publisher('/mlmap', MarkerArray, queue_size=1)
        self.path_viz_pub = rospy.Publisher('/planning/local_path', Marker, queue_size=1)
    
    def set_subscriber_protocol(self):
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
    
    def system_status_cb(self, msg):
        self.system_status = msg.systemState.data 
    
    def vehicle_state_cb(self, msg):
        self.current_velocity = msg.velocity.data
        self.current_signal = msg.signal.data
        self.current_position_lat = msg.position.x
        self.current_position_long = msg.position.y
    
    def publish(self, pos, path):
        if pos == None or path == None:
            return
        self.navigation_data = NavigationData()
        self.navigation_data.currentLocation.x = pos[0]
        self.navigation_data.currentLocation.y = pos[1]
        for xy in path:
            point = Point()
            point.x = xy[0]
            point.y = xy[1]
            self.navigation_data.plannedRoute.append(point)
        
        self.navigation_data_pub.publish(self.navigation_data) 

    def publish_map(self, lmap_viz, mlmap_viz):
        self.lmap_viz_pub.publish(lmap_viz)
        self.mlmap_viz_pub.publish(mlmap_viz)
    
    def publish_path(self, path_viz):
        self.path_viz_pub.publish(path_viz)
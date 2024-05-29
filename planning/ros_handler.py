import rospy

from drive_msgs.msg import *
from geometry_msgs.msg import Point, PoseArray
from visualization_msgs.msg import Marker, MarkerArray

class ROSHandler():
    def __init__(self):
        rospy.init_node('planning', anonymous=False)
        
        self.set_values()
        self.set_publisher_protocol()
        self.set_subscriber_protocol()

    def set_values(self):
        self.system_mode = 0
        self.current_velocity = 0
        self.current_signal = 0
        self.current_position_lat = 0
        self.current_position_long = 0

        self.ltpl_local_path = None
        self.ltpl_local_kappa = None
        self.ltpl_local_vel = None

    def set_publisher_protocol(self):
        self.navigation_data_pub = rospy.Publisher('/NavigationData', NavigationData, queue_size=1)
        self.lmap_viz_pub = rospy.Publisher('/lmap', MarkerArray, queue_size=1)
        self.mlmap_viz_pub = rospy.Publisher('/mlmap', MarkerArray, queue_size=1)
        self.path_viz_pub = rospy.Publisher('/planning/local_path', Marker, queue_size=1)
        self.kappa_viz_pub = rospy.Publisher('/planning/kappa_viz', Marker, queue_size=1)
        
    def set_subscriber_protocol(self):
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        rospy.Subscriber('/ltpl/local_action_set', PoseArray, self.local_action_set_cb)

    def system_status_cb(self, msg):
        self.system_mode = msg.systemMode.data 
        self.current_signal = msg.systemSignal.data
    
    def vehicle_state_cb(self, msg):
        self.current_velocity = msg.velocity.data
        self.current_position_lat = msg.position.x
        self.current_position_long = msg.position.y
    
    def local_action_set_cb(self, msg):
        local_path = []
        local_vel = []
        local_kappa = []

        for pose in msg.poses:
            local_path.append((pose.position.x, pose.position.y))
            local_kappa.append(pose.orientation.y)
            local_vel.append(pose.orientation.z)

        self.ltpl_local_path = local_path
        self.ltpl_local_vel = local_vel
        self.ltpl_local_kappa = local_kappa
    
    def publish(self, pos, path, kappa, velocity):
        if pos == None:
            return
        self.navigation_data = NavigationData()
        self.navigation_data.currentLocation.x = pos[0]
        self.navigation_data.currentLocation.y = pos[1]
        if path != None:
            for xy in path:
                point = Point()
                point.x = xy[0]
                point.y = xy[1]
                self.navigation_data.plannedRoute.append(point)
        if kappa != None:
            for rk in kappa:
                self.navigation_data.plannedKappa.append(rk)
        self.navigation_data.plannedVelocity.data = velocity
        self.navigation_data_pub.publish(self.navigation_data) 
    
    def publish2(self, pos):
        self.navigation_data = NavigationData()
        self.navigation_data.currentLocation.x = pos[0]
        self.navigation_data.currentLocation.y = pos[1]
        if self.ltpl_local_path != None:
            for xy in self.ltpl_local_path:
                point = Point()
                point.x = xy[0]
                point.y = xy[1]
                self.navigation_data.plannedRoute.append(point)
        if self.ltpl_local_kappa != None:
            for rk in self.ltpl_local_kappa:
                self.navigation_data.plannedKappa.append(rk)
        if self.ltpl_local_vel != None:
            self.navigation_data.plannedVelocity.data = self.ltpl_local_vel[0]
        self.navigation_data_pub.publish(self.navigation_data)

    def publish_map(self, lmap_viz, mlmap_viz):
        self.lmap_viz_pub.publish(lmap_viz)
        #self.mlmap_viz_pub.publish(mlmap_viz)
    
    def publish_path(self, path_viz):
        self.path_viz_pub.publish(path_viz)
    
    def publish_kappa(self, path_viz):
        self.kappa_viz_pub.publish(path_viz)
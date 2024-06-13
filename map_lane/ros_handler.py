import rospy

from drive_msgs.msg import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from libs.planning_utils import *
from pyproj import Proj, Transformer

class ROSHandler():
    def __init__(self):
        rospy.init_node('map_lane', anonymous=False)
        
        self.set_values()
        self.set_publisher_protocol()
        self.set_subscriber_protocol()

    def set_values(self):
        self.map_name = None
        self.base_lla = [0,0,0]
        self.system_mode = 0
        self.current_velocity = 0
        self.current_heading = 0
        self.current_signal = 0
        self.local_pos = [0,0]
        self.transformer = None

    def set_publisher_protocol(self):
        self.navigation_data_pub = rospy.Publisher('/NavigationData', NavigationData, queue_size=1)
        self.lmap_viz_pub = rospy.Publisher('/lmap', MarkerArray, queue_size=1)
        self.mlmap_viz_pub = rospy.Publisher('/mlmap', MarkerArray, queue_size=1)
        self.path_viz_pub = rospy.Publisher('/planning/local_path', Marker, queue_size=1)
        self.kappa_viz_pub = rospy.Publisher('/planning/kappa_viz', Marker, queue_size=1)
        
    def set_subscriber_protocol(self):
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)

    def system_status_cb(self, msg):
        self.map_name = msg.mapName.data
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
        if self.transformer == None:
            return
        x, y, _ = self.transformer.transform(msg.position.y, msg.position.x, 7) 
        self.local_pos = [x,y]


    def publish(self, path, kappa, velocity):
        self.navigation_data = NavigationData()
        self.navigation_data.currentLocation.x = self.local_pos[0]
        self.navigation_data.currentLocation.y = self.local_pos[1]
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

    def publish_map_viz(self, lmap_viz, mlmap_viz):
        self.lmap_viz_pub.publish(lmap_viz)
        #self.mlmap_viz_pub.publish(mlmap_viz)
    

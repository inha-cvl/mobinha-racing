import rospy

from drive_msgs.msg import *
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from libs.planning_utils import *

class ROSHandler():
    def __init__(self, base_lla):
        rospy.init_node('planning', anonymous=False)
        
        self.set_values()
        self.set_publisher_protocol()
        self.set_subscriber_protocol()
        self.base_lla = base_lla

    def set_values(self):
        self.system_mode = 0
        self.current_velocity = 0
        self.current_heading = 0
        self.current_signal = 0
        self.current_position_lat = 0
        self.current_position_long = 0
        self.local_pos = [0,0]
        self.prev_start_pos = [0,0] 

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
        self.system_mode = msg.systemMode.data 
        self.current_signal = msg.systemSignal.data
    
    def vehicle_state_cb(self, msg):
        self.current_velocity = msg.velocity.data
        self.current_heading = msg.heading.data
        self.current_position_lat = msg.position.x
        self.current_position_long = msg.position.y
        x, y, _ = geodetic2enu(msg.position.x, msg.position.y, 7, self.base_lla[0], self.base_lla[1], self.base_lla[2])
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
            if path[0] != self.prev_start_pos:
                self.publish_path_viz(path)
                self.prev_start_pos = path[0]
        if kappa != None:
            for rk in kappa:
                self.navigation_data.plannedKappa.append(rk)
        self.navigation_data.plannedVelocity.data = velocity
        self.navigation_data_pub.publish(self.navigation_data)
    
    def publish2(self, local_action_set):
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

    def publish_map_viz(self, lmap_viz, mlmap_viz):
        self.lmap_viz_pub.publish(lmap_viz)
        #self.mlmap_viz_pub.publish(mlmap_viz)
    
    def publish_path_viz(self, paths):
        if paths == None:
            return
        path_viz = self.LocalPathViz(paths)
        self.path_viz_pub.publish(path_viz)
    
    def publish_kappa_viz(self, kappas):
        kappa_viz = self.KappaPathViz(kappas)
        self.kappa_viz_pub.publish(kappa_viz)
    
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
        

import rospy
import math

from drive_msgs.msg import *
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from jsk_recognition_msgs.msg import BoundingBoxArray
from std_msgs.msg import String 

from libs.planning_utils import *
from pyproj import Proj, Transformer
from libs.obstalce_handler import ObstacleHandler


class ROSHandler():
    def __init__(self):
        rospy.init_node('map_lane', anonymous=False)
        
        self.set_values()
        self.set_publisher_protocol()
        self.set_subscriber_protocol()

    def set_values(self):
        self.oh = ObstacleHandler()
        self.map_name = None
        self.base_lla = [0,0,0]
        self.system_mode = 0
        self.current_velocity = 0
        self.current_heading = 0
        self.current_signal = 0
        self.local_pose = [0,0]
        self.transformer = None

        self.sim_obstacles = [] # cls, x, y, v
        self.cam_obstacles = []
        self.lid_obstacles = []

    def set_publisher_protocol(self):
        self.navigation_data_pub = rospy.Publisher('/NavigationData', NavigationData, queue_size=1)
        self.lmap_viz_pub = rospy.Publisher('/lmap', MarkerArray, queue_size=10, latch=True)
        self.mlmap_viz_pub = rospy.Publisher('/mlmap', MarkerArray, queue_size=10, latch=True)
        self.path_viz_pub = rospy.Publisher('/planning/local_path', Marker, queue_size=1)
        self.kappa_viz_pub = rospy.Publisher('/planning/kappa_viz', Marker, queue_size=1)
        self.lane_data_pub = rospy.Publisher('/LaneData', LaneData, queue_size=1)
        self.lanelet_pub = rospy.Publisher('/LaneLet', LaneLet, queue_size=1)
        self.refine_obstacles_pub = rospy.Publisher('/map_lane/refine_obstacles', PoseArray, queue_size=1)

    def set_subscriber_protocol(self):
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)

        rospy.Subscriber('/detection_markers', MarkerArray, self.cam_objects_cb)
        rospy.Subscriber('/perception/box_detection', MarkerArray, self.cam_box_objects_cb)
        rospy.Subscriber('/mobinha/perception/lidar/track_box', BoundingBoxArray, self.lidar_track_box_cb)
        rospy.Subscriber('/simulator/objects', PoseArray, self.sim_objects_cb)
        
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
        self.local_pose = [x,y]
        self.oh.update_value(self.local_pose, self.current_heading)

    def sim_objects_cb(self, msg):
        obstacles = []
        for obj in msg.poses:
            obstacles.append([int(obj.position.z),obj.position.x,obj.position.y,float(obj.orientation.x)])
        self.sim_obstacles = obstacles

    
    def cam_objects_cb(self,msg):
        obstacles = []
        for obj in msg.markers:
            conv = self.oh.object2enu([obj.pose.position.x, obj.pose.position.y])
            if conv is None:
                continue
            else:
                nx,ny = conv
                obstacles.append([0, nx, ny, 3])
        self.cam_obstacles = obstacles
    
    def cam_box_objects_cb(self,msg):
        obstacles = []
        for obj in msg.markers:
            conv = self.oh.object2enu([obj.pose.position.x, obj.pose.position.y])
            if conv is None:
                continue
            else:
                nx,ny = conv
                obstacles.append([0, nx, ny, 3])
        self.cam_obstacles = obstacles

    def lidar_track_box_cb(self, msg):
        obstacles = []
        for obj in msg.boxes:
            conv = self.oh.object2enu([obj.pose.position.x, obj.pose.position.y])
            if conv is None:
                return
            else:
                nx,ny = conv
                obstacles.append([0, nx, ny, 3])
        self.lid_obstacles = obstacles 

    def publish(self, path, kappa, velocity):
        self.navigation_data = NavigationData()
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
        self.mlmap_viz_pub.publish(mlmap_viz)
    
    def publish_lane_data(self, curr_lane_num, curr_lane_id):
        laneLet = LaneLet()
        laneLet.id.data = str(curr_lane_id)
        laneLet.laneNumber.data = int(curr_lane_num)

        laneData = LaneData()
        laneData.currentLane = laneLet

        self.lane_data_pub.publish(laneData)
    
    def publish_refine_obstacles(self, obstacles):
        pose_array = PoseArray()
        for obs in obstacles:
            pose = Pose()
            pose.position.x = obs[1]
            pose.position.y = obs[2]
            pose.position.z = obs[0]
            pose.orientation.x = obs[3]
            pose.orientation.y = obs[4]
            pose_array.poses.append(pose)
        self.refine_obstacles_pub.publish(pose_array)
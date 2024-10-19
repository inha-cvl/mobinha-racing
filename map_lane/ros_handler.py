import rospy
import math

from drive_msgs.msg import *
from geometry_msgs.msg import Point, Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from jsk_recognition_msgs.msg import BoundingBoxArray
from std_msgs.msg import Float32MultiArray, Int8MultiArray


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
        self.est_veh_spd = 0

        self.sim_obstacles = [] # cls, x, y, v
        self.cam_obstacles = []
        self.lid_obstacles = []
        self.rad_obstacles = []
        self.fus_obstacles = []

    def set_publisher_protocol(self):
        self.navigation_data_pub = rospy.Publisher('/NavigationData', NavigationData, queue_size=1)
        self.lmap_viz_pub = rospy.Publisher('/lmap', MarkerArray, queue_size=10, latch=True)
        self.mlmap_viz_pub = rospy.Publisher('/mlmap', MarkerArray, queue_size=10, latch=True)
        self.path_viz_pub = rospy.Publisher('/planning/local_path', Marker, queue_size=1)
        self.kappa_viz_pub = rospy.Publisher('/planning/kappa_viz', Marker, queue_size=1)
        self.lane_data_pub = rospy.Publisher('/LaneData', LaneData, queue_size=1)
        self.lanelet_pub = rospy.Publisher('/LaneLet', LaneLet, queue_size=1)
        self.refine_obstacles_pub = rospy.Publisher('/map_lane/refine_obstacles', PoseArray, queue_size=1)
        self.lidar_bsd_pub = rospy.Publisher('/map_lane/lidar_bsd', Int8MultiArray,queue_size=1)

    def set_subscriber_protocol(self):
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        rospy.Subscriber('/ADAS_DRV',Float32MultiArray, self.adas_drv_cb)

        rospy.Subscriber('/mobinha/perception/lidar/track_box', BoundingBoxArray, self.lidar_track_box_cb)
        rospy.Subscriber('/simulator/objects', PoseArray, self.sim_objects_cb)
        #rospy.Subscriber('/RadarObjectArray', RadarObjectArray, self.radar_object_array_cb)
        rospy.Subscriber('/perception/fusion_objects', PoseArray, self.fusion_objects_cb)

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
    
    def adas_drv_cb(self, msg):
        self.est_veh_spd = msg.data[5]

    def radar_object_array_cb(self, msg):
        rad_obstacles = []
        for ro in msg.radarObjects:
            if ro.mvngFlag.data > 0 and ro.qualLvl.data > 32  and ro.coastAge.data < 1 and ro.alvAge.data > 2:
                obj = [ro.relPosX.data+2.325, ro.relPosY.data, ro.relVelX.data, ro.relVelY.data, ro.alvAge.data]
                rad_obstacles.append(obj)

        clustered_obstacles = self.oh.cluster_radar_obstacles(rad_obstacles)
        
        obstacles = []
        for co in clustered_obstacles:
            conv = self.oh.object2enu([co[0],co[1]])
            if conv is None:
                continue
            else:
                nx, ny = conv
                obstacles.append([0, nx, ny, co[3]+self.current_velocity]) # id, x, y, vel
        self.rad_obstacles = obstacles

    def sim_objects_cb(self, msg):
        obstacles = []
        for obj in msg.poses:
            obstacles.append([int(obj.position.z),obj.position.x,obj.position.y,float(obj.orientation.x)])
        self.sim_obstacles = obstacles
        
    
    def fusion_objects_cb(self, msg):
        obstacles = []
        for obj in msg.poses:
            conv = self.oh.object2enu([obj.position.x,obj.position.y])
            if conv is None:
                continue
            else:
                nx, ny = conv
                obstacles.append([int(obj.position.z),nx, ny,float(obj.orientation.x)])
        self.fus_obstacles = obstacles

    def lidar_track_box_cb(self, msg):
        obstacles = []
        bsd_msg = Int8MultiArray()
        bsd_msg.data = [0,0]
        car_x_array = []
        car_y_array = []

        for obj in msg.boxes:
            
            if obj.header.seq < 3:
                continue
            
            car_x = obj.pose.position.x
            car_y = obj.pose.position.y
            car_x_array.append(car_x)
            car_y_array.append(car_y)

            offset = 20

            if any(-offset< x for x in car_x_array) & any(offset> x for x in car_x_array) & any(2 < y for y in car_y_array) & any(5 > y for y in car_y_array):
                bsd_msg.data[0] = 1
            else:
                bsd_msg.data[0] = 0

            if any(-offset< x for x in car_x_array) & any(offset> x for x in car_x_array) & any(-5 < y for y in car_y_array) & any(-2 > y for y in car_y_array):
                bsd_msg.data[1] = 1
            else:
                bsd_msg.data[1] = 0

            conv = self.oh.object2enu([obj.pose.position.x, obj.pose.position.y])
            if conv is None:
                return
            else:
                nx,ny = conv
                v =  (obj.value if obj.value != 0 else 0 ) + self.current_velocity
                obstacles.append([0, nx, ny, v])
        self.lid_obstacles = obstacles 
        self.lidar_bsd_pub.publish(bsd_msg)

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
        self.navigation_data.targetVelocity.data = velocity
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
            pose.position.x = obs[1] #pos x
            pose.position.y = obs[2] #pos y 
            pose.position.z = obs[0] #type
            pose.orientation.x = obs[3] #velocity
            pose.orientation.y = obs[4] #heading
            pose.orientation.z = obs[5] #distance
            pose_array.poses.append(pose)
        self.refine_obstacles_pub.publish(pose_array)
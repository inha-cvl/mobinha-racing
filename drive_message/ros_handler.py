import rospy
import numpy as np

from drive_msgs.msg import *
from geometry_msgs.msg import QuaternionStamped, PoseArray, Pose2D
from nmea_msgs.msg import Sentence
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix
from jsk_recognition_msgs.msg import BoundingBoxArray
from visualization_msgs.msg import MarkerArray

from libs.message_handler import *
from libs.obstalce_handler import ObstacleHandler

USE_LIDAR = True

class ROSHandler():
    def __init__(self, map, base_lla):
        rospy.init_node('drive_message', anonymous=False)
        self.set_messages(map, base_lla)
        self.set_values()
        self.set_publisher_protocol()
        self.set_subscriber_protocol()

    def set_messages(self, map, base_lla):
        self.can_input = CANInput()
        self.can_input.EPS_Speed.data = 50 # 80 : 15
        self.sensor_data = SensorData()
        self.system_status = SystemStatus()
        self.system_status.mapName.data = map
        self.system_status.baseLLA = base_lla
        self.vehicle_state = VehicleState()
        self.lane_data = LaneData()
        self.detection_data = DetectionData()
        self.ego_actuator = Actuator()

    def set_values(self):
        self.oh = ObstacleHandler()
        self.ego_local_pose = []
        self.prev_lla = None
        self.lap_cnt = 0
        self.start_point = (0, 0, 0)
        self.lane_number = 0
        self.planned_route = []
        self.route_length = 100
        self.offset = 1.5

    def set_publisher_protocol(self):
        self.sensor_data_pub = rospy.Publisher('/SensorData', SensorData, queue_size=1)
        self.system_status_pub = rospy.Publisher('/SystemStatus', SystemStatus, queue_size=1)
        self.vehicle_state_pub = rospy.Publisher('/VehicleState', VehicleState, queue_size=1)
        self.detection_data_pub = rospy.Publisher('/DetectionData', DetectionData, queue_size=1)
        self.can_input_pub = rospy.Publisher('/CANInput', CANInput, queue_size=1)
        self.ego_actuator_pub = rospy.Publisher('/EgoActuator', Actuator, queue_size=1)
    
    def set_subscriber_protocol(self):
        rospy.Subscriber('/CANOutput', CANOutput, self.can_output_cb)
        rospy.Subscriber('/NavigationData', NavigationData, self.navigation_data_cb)
        rospy.Subscriber('/UserInput', UserInput, self.user_input_cb)
        rospy.Subscriber('/control/target_actuator', Actuator, self.target_actuator_cb)
        rospy.Subscriber('LaneData', LaneData, self.lane_data_cb)
        rospy.Subscriber('/nmea_sentence', Sentence, self.nmea_sentence_cb)
        rospy.Subscriber('/fix', NavSatFix, self.nav_sat_fix_cb)
        rospy.Subscriber('/heading', QuaternionStamped, self.heading_cb)

        if not USE_LIDAR:
            rospy.Subscriber('/detection_markers', MarkerArray, self.cam_objects_cb)
        else:
            rospy.Subscriber('/mobinha/perception/lidar/track_box', BoundingBoxArray, self.lidar_track_box_cb)
        
        # Simulator
        rospy.Subscriber('/simulator/pose', Pose2D, self.sim_pose_cb)
        rospy.Subscriber('/simulator/objects', PoseArray, self.sim_objects_cb)

    def can_output_cb(self, msg):
        self.vehicle_state.mode.data = mode_checker(msg.EPS_Control_Status.data, msg.ACC_Control_Status.data)  # off, on, steering, acc/brake
        self.vehicle_state.velocity.data = calc_wheel_velocity(msg.WHEEL_SPD_RR.data, msg.WHEEL_SPD_RL.data)  # m/s
        self.vehicle_state.gear.data = str(msg.G_SEL_DISP.data)    
        self.vehicle_state.signal.data = turn_signal_checker(msg.Turn_Left_En.data, msg.Turn_Right_En.data)  # blinker 같음
        self.ego_actuator.accel.data = float(msg.Long_ACCEL.data)
        self.ego_actuator.brake.data = float(msg.BRK_CYLINDER.data)
        self.ego_actuator.steer.data = float(msg.StrAng.data)
    
    def navigation_data_cb(self, msg):
        self.ego_local_pose = (msg.currentLocation.x, msg.currentLocation.y, msg.currentLocation.z)
        for point in msg.plannedRoute:
            self.planned_route.append(point)
            if len(self.planned_route) > self.route_length:
                self.planned_route.pop(0)
        self.lap_cnt, self.lap_flag = check_lap_count(self.lap_cnt, self.ego_local_pose, self.start_point, radius=20, lap_flag=False)
        self.system_status.lapCount.data = self.lap_cnt

    def lane_data_cb(self, msg:LaneData):
        if msg.currentLane.currentLane != 0:
            self.lane_number = msg.currentLane.currentLane
        
    def signal_cb(self, msg):
        self.system_status.systemSignal.data = int(msg.data)

    def user_input_cb(self, msg): #mode, signal, state, health
        mode = int(msg.user_mode.data)
        self.system_to_can(mode)
        self.system_status.systemMode.data = mode
        self.system_status.systemSignal.data = int(msg.user_signal.data)

    def nmea_sentence_cb(self, msg):
        self.vehicle_state.header = msg.header
        if self.prev_lla is None:
            parsed = nmea_parser(0, 0, msg.sentence) 
        else:
            parsed = nmea_parser(self.prev_lla[0], self.prev_lla[1], msg.sentence)
        if parsed != None:
            if len(parsed) == 3:
                self.vehicle_state.position.x = parsed[0]
                self.vehicle_state.position.y = parsed[1]
                if self.prev_lla is None:
                    self.prev_lla = (parsed[0], parsed[1])
                    return
                else: 
                    self.vehicle_state.heading.data = parsed[2]
            elif len(parsed) == 1:
                self.vehicle_state.heading.data = parsed[0]        
    
    def sim_pose_cb(self, msg):
        self.vehicle_state.header = Header()
        self.vehicle_state.header.stamp = rospy.Time.now()
        self.vehicle_state.position.x = msg.x
        self.vehicle_state.position.y = msg.y
        self.vehicle_state.heading.data = msg.theta

    def nav_sat_fix_cb(self, msg):  # nmea_sentence error handling
        self.vehicle_state.header = msg.header
        if not check_error(self.vehicle_state.position.x, msg.latitude, 30):
            self.vehicle_state.position.x = msg.latitude
        if not check_error(self.vehicle_state.position.y, msg.longitude, 30):
            self.vehicle_state.position.y = msg.longitude
    
    def heading_cb(self, msg):
        yaw = match_heading(msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w)
        # if not check_error(self.vehicle_state.heading.data, yaw, 90):
        self.vehicle_state.heading.data = yaw  

    def target_actuator_cb(self, msg):
        steer = np.clip(msg.steer.data*12.9, -500, 500)
        self.can_input.EPS_Cmd.data = steer
        self.can_input.ACC_Cmd.data = msg.accel.data if msg.accel.data > 0 else -msg.brake.data
    
    def sim_objects_cb(self, msg):
        self.detection_data = DetectionData()
        for obj in msg.poses:
            object_info = ObjectInfo()
            object_info.type.data = int(obj.position.z)
            object_info.position.x = float(obj.position.x)
            object_info.position.y = float(obj.position.y)
            object_info.velocity.data = float(obj.orientation.x)
            object_info.heading.data = float(obj.orientation.y)
            self.detection_data.objects.append(object_info)
    
    def cam_objects_cb(self,msg):
        self.detection_data = DetectionData()
        for obj in msg.markers:
            object_info = ObjectInfo()
            object_info.type.data = 0
            conv = self.oh.object2enu([obj.pose.position.x, obj.pose.position.y])
            if conv is None:
                return
            else:
                nx,ny = conv
                object_info.position.x = nx
                object_info.position.y = ny
                object_info.velocity.data = self.vehicle_state.velocity.data
                object_info.heading.data = self.vehicle_state.heading.data
                self.detection_data.objects.append(object_info)
        
    def lidar_track_box_cb(self, msg):
        self.detection_data = DetectionData()
        for obj in msg.boxes:
            object_info = ObjectInfo()
            object_info.type.data = 0
            conv = self.oh.object2enu([obj.pose.position.x, obj.pose.position.y])
            if conv is None:
                return
            else:
                nx,ny = conv
                fred_d = self.oh.object2frenet([nx, ny])
                if not self.oh.filtering_by_lane_num(self.lane_number, fred_d):
                    continue                    
                object_info.position.x = nx
                object_info.position.y = ny
                object_info.velocity.data = obj.value
                object_info.heading.data = self.oh.get_absolute_heading(self.vehicle_state.heading.data, obj.pose.orientation)
                self.detection_data.objects.append(object_info)
            
    def system_to_can(self, mode):
        if self.vehicle_state.mode.data == 0:
            self.can_input.EPS_En.data = 1 if mode in (1, 2) else 0
            self.can_input.ACC_En.data = 1 if mode in (1, 3) else 0

        elif mode == 0 and self.vehicle_state.mode.data >= 1:
            self.can_input.EPS_En.data = 0
            self.can_input.ACC_En.data = 0
        
    def publish(self):
        self.can_input_pub.publish(self.can_input)
        self.sensor_data_pub.publish(self.sensor_data)
        self.system_status_pub.publish(self.system_status)
        self.vehicle_state_pub.publish(self.vehicle_state)
        self.detection_data_pub.publish(self.detection_data)
        self.ego_actuator_pub.publish(self.ego_actuator)
        self.oh.update_value(self.ego_local_pose, self.vehicle_state.heading.data)
# ros_handler.py
import rospy
import numpy as np

from drive_msgs.msg import *
from geometry_msgs.msg import QuaternionStamped, PoseArray
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix
from jsk_recognition_msgs.msg import BoundingBoxArray
from visualization_msgs.msg import MarkerArray

from libs.message_handler import *
from libs.sensor_check import *
from sensor_status_handler import SensorStatusHandler

USE_LIDAR = True

class ROSHandler():
    def __init__(self, map, base_lla):
        rospy.init_node('drive_message', anonymous=False)
        self.set_messages(map, base_lla)
        self.set_publisher_protocol()
        self.set_subscriber_protocol()

        #TODO check system status
        # self.gps_check = FrequencyCheck('/fix', NavSatFix, 1.0)
        # if not USE_LIDAR:
        #     self.detection_check = FrequencyCheck('/detection_markers', MarkerArray, 1.0)
        # else:
        #     self.detection_check = FrequencyCheck('/mobinha/perception/lidar/track_box', BoundingBoxArray, 1.0) 
        # Soon add other sensors

        self.sensor_status_handler = SensorStatusHandler()
        rospy.Timer(rospy.Duration(0.5), self.check_sensor_status1R)

    def set_messages(self, map, base_lla):
        self.can_input = CANInput()
        self.can_input.EPS_Speed.data = 50
        self.ego_local_pose = []
        self.sensor_data = SensorData()
        self.system_status = SystemStatus()
        self.system_status.mapName.data = map
        self.system_status.baseLLA = base_lla
        self.vehicle_state = VehicleState()
        self.lane_data = LaneData()
        self.detection_data = DetectionData()
        self.ego_actuator = Actuator()

        self.prev_lla = None

        self.lap_cnt = 0
        self.start_point = (0, 0, 0)
        self.radius = 20
        self.lap_flag = False

        self.planned_route = []
        self.route_length = 100
        self.offset = 1.5

    def set_publisher_protocol(self):
        self.sensor_data_pub = rospy.Publisher('/SensorData', SensorData, queue_size=1)
        self.system_status_pub = rospy.Publisher('/SystemStatus', SystemStatus, queue_size=1)
        self.vehicle_state_pub = rospy.Publisher('/VehicleState', VehicleState, queue_size=1)
        self.lane_data_pub = rospy.Publisher('/LaneData', LaneData, queue_size=1)
        self.detection_data_pub = rospy.Publisher('/DetectionData', DetectionData, queue_size=1)
        self.can_input_pub = rospy.Publisher('/CANInput', CANInput, queue_size=1)
        self.ego_actuator_pub = rospy.Publisher('/EgoActuator', Actuator, queue_size=1)
    
    def set_subscriber_protocol(self):
        rospy.Subscriber('/CANOutput', CANOutput, self.can_output_cb)
        rospy.Subscriber('/NavigationData', NavigationData, self.navigation_data_cb)
        rospy.Subscriber('/simulator/nmea_sentence', Sentence, self.sim_nmea_sentence_cb)
        rospy.Subscriber('/UserInput', UserInput, self.user_input_cb)
        rospy.Subscriber('/control/target_actuator', Actuator, self.target_actuator_cb)
        rospy.Subscriber('/fix', NavSatFix, self.nav_sat_fix_cb)
        rospy.Subscriber('/heading', QuaternionStamped, self.heading_cb)
        rospy.Subscriber('/simulator/objects', PoseArray, self.sim_objects_cb)

        if not USE_LIDAR:
            rospy.Subscriber('/detection_markers', MarkerArray, self.cam_objects_cb)
        else:
            rospy.Subscriber('/mobinha/perception/lidar/track_box', BoundingBoxArray, self.lidar_track_box_cb)

    def can_output_cb(self, msg):
        self.vehicle_state.mode.data = mode_checker(msg.EPS_Control_Status.data, msg.ACC_Control_Status.data)
        self.vehicle_state.velocity.data = calc_wheel_velocity(msg.WHEEL_SPD_RR.data, msg.WHEEL_SPD_RL.data)
        self.vehicle_state.gear.data = str(msg.G_SEL_DISP.data)    
        self.vehicle_state.signal.data = turn_signal_checker(msg.Turn_Left_En.data, msg.Turn_Right_En.data)
        self.ego_actuator.accel.data = float(msg.Long_ACCEL.data)
        self.ego_actuator.brake.data = float(msg.BRK_CYLINDER.data)
        self.ego_actuator.steer.data = float(msg.StrAng.data)

    def navigation_data_cb(self, msg):
        self.ego_local_pose = (msg.currentLocation.x, msg.currentLocation.y, msg.currentLocation.z)
        for point in msg.plannedRoute:
            self.planned_route.append(point)
            if len(self.planned_route) > self.route_length:
                self.planned_route.pop(0)
        self.lap_cnt, self.lap_flag = check_lap_count(self.lap_cnt, self.ego_local_pose, self.start_point, self.radius, self.lap_flag)
        self.system_status.lapCount.data = self.lap_cnt
        
    def signal_cb(self, msg):
        self.system_status.systemSignal.data = int(msg.data)

    def user_input_cb(self, msg):
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
    
    def sim_nmea_sentence_cb(self, msg):
        self.vehicle_state.header = msg.header
        parsed = sim_nmea_parser(msg.sentence)
        if parsed != None:
            if len(parsed) == 2:
                self.vehicle_state.position.x = parsed[0]
                self.vehicle_state.position.y = parsed[1]
            elif len(parsed) == 1:
                self.vehicle_state.heading.data = parsed[0]

    def nav_sat_fix_cb(self, msg):
        self.vehicle_state.header = msg.header
        if not check_error(self.vehicle_state.position.x, msg.latitude, 30):
            self.vehicle_state.position.x = msg.latitude
        if not check_error(self.vehicle_state.position.y, msg.longitude, 30):
            self.vehicle_state.position.y = msg.longitude
    
    def heading_cb(self, msg):
        yaw = match_heading(msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w)
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
            conv = convert_local_to_enu(self.ego_local_pose, self.vehicle_state.heading.data, (obj.pose.position.x, obj.pose.position.y))
            if conv is None:
                return
            else:
                x,y = conv
            object_info.position.x = x
            object_info.position.y = y
            object_info.velocity.data = self.vehicle_state.velocity.data
            object_info.heading.data = self.vehicle_state.heading.data
            self.detection_data.objects.append(object_info)
    
    def lidar_track_box_cb(self, msg):
        self.detection_data = DetectionData()
        for obj in msg.boxes:
            object_info = ObjectInfo()
            object_info.type.data = 0
            conv = convert_local_to_enu(self.ego_local_pose, self.vehicle_state.heading.data, (obj.pose.position.x, obj.pose.position.y))
            if conv is None:
                return
            else:
                x,y = conv
                for route_point in self.planned_route:
                    distance_to_route = distance((x, y), (route_point.x, route_point.y))
                    if distance_to_route <= self.offset:
                        q = obj.pose.orientation
                        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                        z_angle_rad = np.arctan2(siny_cosp, cosy_cosp)
                        z_angle_deg = np.degrees(z_angle_rad)
                        object_info.position.x = x
                        object_info.position.y = y
                        object_info.velocity.data = obj.value
                        object_info.heading.data = self.vehicle_state.heading.data - z_angle_deg
                        self.detection_data.objects.append(object_info)
                        break
            
    def system_to_can(self, mode):
        if self.vehicle_state.mode.data == 0:
            self.can_input.EPS_En.data = 1 if mode in (1, 2) else 0
            self.can_input.ACC_En.data = 1 if mode in (1, 3) else 0

        elif mode == 0 and self.vehicle_state.mode.data >= 1:
            self.can_input.EPS_En.data = 0
            self.can_input.ACC_En.data = 0

    #TODO check sensor state for system health(temp) 
    # def check_sensor_state(self):
    #     system_health = 0  # 0: OK, 1: Warning, 2: Error
    #     if not self.gps_check.check():
    #         system_health == 2
    #     if not self.detection_check.check():
    #         system_health == 1
    #     self.system_status.systemHealth.data = system_health

    def check_sensor_status1R(self):
        system_health = 0  # 0: OK, 1: Warning, 2: Error
        if not self.sensor_status_handler.check_camera_status():
            rospy.logwarn("Camera is not working properly")
            if self.lap_cnt == 0:
                system_health = 2
            elif self.lap_cnt >= 1:
                system_health = 1
        if not self.sensor_status_handler.check_lidar_status():
            rospy.logwarn("LiDAR is not working properly")
            if self.lap_cnt == 0:
                system_health = 2
            elif self.lap_cnt >= 1:
                system_health = 1
        if not self.sensor_status_handler.check_gps_status():
            rospy.logwarn("GPS is not working properly")
            system_health = 2
        if not self.sensor_status_handler.check_can_status():
            rospy.logwarn("CAN is not working properly")
            system_health = 2

        self.system_status.systemHealth.data = system_health

    def check_sensor_status2R(self):
        system_health = 0  # 0: OK, 1: Warning, 2: Error
        if not self.sensor_status_handler.check_camera_status():
            rospy.logwarn("Camera is not working properly")
            system_health = 2
        if not self.sensor_status_handler.check_lidar_status():
            rospy.logwarn("LiDAR is not working properly")
            system_health = 2
        if not self.sensor_status_handler.check_gps_status():
            rospy.logwarn("GPS is not working properly")
            system_health = 2
        if not self.sensor_status_handler.check_can_status():
            rospy.logwarn("CAN is not working properly")
            system_health = 2

        self.system_status.systemHealth.data = system_health

    def check_sensor_statusFinal(self):
        system_health = 0  # 0: OK, 1: Warning, 2: Error

        self.system_status.systemHealth.data = system_health

    def publish(self):
        self.can_input_pub.publish(self.can_input)
        self.sensor_data_pub.publish(self.sensor_data)
        self.system_status_pub.publish(self.system_status)
        self.vehicle_state_pub.publish(self.vehicle_state)
        self.lane_data_pub.publish(self.lane_data)
        self.detection_data_pub.publish(self.detection_data)
        self.ego_actuator_pub.publish(self.ego_actuator)

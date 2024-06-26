import rospy
import numpy as np

from drive_msgs.msg import *
from geometry_msgs.msg import QuaternionStamped, PoseArray
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8MultiArray

from libs.message_handler import *

from statemachine import StateMachine, State

class SpeedMachine(StateMachine):
    idle = State(initial=True)
    keeping_speed = State()
    acceleration = State()
    deceleration = State()
    emergency_stop = State(final=True)

    accel_event = (
        acceleration.to.itself(internal=True)
        | idle.to(acceleration, cond="accel_on_straight")
        | keeping_speed.to(acceleration, cond="accel_on_straight")
        | keeping_speed.to(acceleration, cond="accel_on_curve")
        | keeping_speed.to(acceleration, cond="accel_on_bank")
        | deceleration.to(acceleration, cond="accel_on_straight")
        | deceleration.to(acceleration, cond="accel_on_curve")
        | deceleration.to(acceleration, cond="accel_on_bank")
    )
    
    keep_event = (
        keeping_speed.to.itself(internal=True)
        | acceleration.to(keeping_speed, cond="keep_on_straight")
        | acceleration.to(keeping_speed, cond="keep_on_curve")
        | acceleration.to(keeping_speed, cond="keep_on_bank")
        | deceleration.to(keeping_speed, cond="keep_on_straight")
        | deceleration.to(keeping_speed, cond="keep_on_curve")
        | deceleration.to(keeping_speed, cond="keep_on_bank")
    )

    decel_event = (
        deceleration.to.itself(internal=True)
        | keeping_speed.to(deceleration, cond="decel_on_straight")
        | keeping_speed.to(deceleration, cond="decel_on_curve")
        | keeping_speed.to(deceleration, cond="decel_on_bank")
        | acceleration.to(deceleration, cond="decel_on_straight")
        | acceleration.to(deceleration, cond="decel_on_curve")
        | acceleration.to(deceleration, cond="decel_on_bank")
    )

    emergency_event = (
        acceleration.to(emergency_stop, cond="emergency")
        | keeping_speed.to(emergency_stop, cond="emergency")
        | deceleration.to(emergency_stop, cond="emergency")
    )

    def __init__(self):
        self.velocity = 0  # VehicleState
        self.road_type = 0  # NavigationData 
        self.front_obstacle = False  # DetectionData
        self.system_health = True
        self.currentLane = 3  # LaneData & DetectionData but, 뱅크구간에서 3차로로만 달릴거면 필요 없음

        super().__init__()

    def proper_speed(self, min_vel, max_vel):
        return min_vel <= self.velocity <= max_vel

    def accel_on_straight(self):  
        return self.road_type == 0 and self.velocity < 75
    
    def keep_on_straight(self):
        return self.road_type == 0 and self.proper_speed(75, 80)
    
    def decel_on_straight(self):
        return (self.road_type == 0 and self.velocity > 80) or self.front_obstacle == True
    
    def accel_on_curve(self):
        return self.road_type == 1 and self.velocity < 55
    
    def keep_on_curve(self):
        return self.road_type == 1 and self.proper_speed(55, 60)
    
    def decel_on_curve(self):
        return self.road_type == 1 and self.velocity > 60
    
    def accel_on_bank(self):
        return self.road_type == 2 and self.velocity < 55

    def keep_on_bank(self):
        return self.road_type == 2 and self.proper_speed(55, 60)
    
    def decel_on_bank(self):
        return self.road_type == 2 and self.velocity > 60
    
    def emergency(self):
        return self.system_health == False

    def on_enter_acceleration(self):
        if self.road_type == 0:
            rospy.loginfo(f"Acceleration on straight road: state=0, max_vel=77")
            self.publish_speedmachine(0, 77)  # Planning 모듈에 메시지 전달 [state_value, max_vel]

        elif self.road_type == 1:
            rospy.loginfo(f"Acceleration on curve road: state=0, max_vel=57")
            self.publish_speedmachine(0, 57)
            
        elif self.road_type == 2:
            rospy.loginfo(f"Acceleration on bank road: state=0, max_vel=57")
            self.publish_speedmachine(0, 57)

    def on_enter_keeping_speed(self):
        rospy.loginfo("Start keeping speed")
        if self.road_type == 0:
            rospy.loginfo(f"Keeping speed on straight road: state=1, max_vel=77")
            self.publish_speedmachine(1, 77)
            
        elif self.road_type == 1:
            rospy.loginfo(f"Keeping speed on curve road: state=1, max_vel=57")
            self.publish_speedmachine(1, 57)
            
        elif self.road_type == 2:
            rospy.loginfo(f"Keeping speed on bank road: state=1, max_vel=57")
            self.publish_speedmachine(1, 57)
    
    def on_exit_keeping_speed(self):
        rospy.loginfo("Finish keeping speed")

    def on_enter_deceleration(self):
        if self.road_type == 0:
            rospy.loginfo(f"Deceleration on straight road: state=2, max_vel=77")
            self.publish_speedmachine(2, 77)
            
        elif self.road_type == 1:
            rospy.loginfo(f"Deceleration on curve road: state=2, max_vel=57")
            self.publish_speedmachine(2, 57)
            
        elif self.road_type == 2:
            rospy.loginfo(f"Deceleration on bank road: state=2, max_vel=57")
            self.publish_speedmachine(2, 57)
            
    def on_enter_emergency(self):
        rospy.loginfo("Emergency! Shut down the system")
        self.publish_speedmachine(9, 0)

class ROSHandler():
    def __init__(self, map, base_lla):
        rospy.init_node('drive_message', anonymous=False)

        self.speed_machine = SpeedMachine()
        self.set_messages(map, base_lla)
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

    def set_publisher_protocol(self):
        self.sensor_data_pub = rospy.Publisher('/SensorData', SensorData, queue_size=1)
        self.system_status_pub = rospy.Publisher('/SystemStatus', SystemStatus, queue_size=1)
        self.vehicle_state_pub = rospy.Publisher('/VehicleState', VehicleState, queue_size=1)
        self.lane_data_pub = rospy.Publisher('/LaneData', LaneData, queue_size=1)
        self.detection_data_pub = rospy.Publisher('/DetectionData', DetectionData, queue_size=1)
        self.can_input_pub = rospy.Publisher('/CANInput', CANInput, queue_size=1)
        self.ego_actuator_pub = rospy.Publisher('/EgoActuator', Actuator, queue_size=1)

        self.speedmachine_pub = rospy.Publisher('/SpeedMachine', Int8MultiArray, queue_size=1)

    def set_subscriber_protocol(self):
        rospy.Subscriber('/CANOutput', CANOutput, self.can_output_cb)
        rospy.Subscriber('/simulator/nmea_sentence', Sentence, self.nmea_sentence_cb)
        rospy.Subscriber('/UserInput', UserInput, self.user_input_cb)
        rospy.Subscriber('/control/target_actuator', Actuator, self.target_actuator_cb)
        rospy.Subscriber('/fix', NavSatFix, self.nav_sat_fix_cb)
        rospy.Subscriber('/heading', QuaternionStamped, self.heading_cb)
        rospy.Subscriber('/simulator/objects', PoseArray, self.objects_cb)

        #rospy.Subscriber('/vehicleState', VehicleState, self.vehicle_state_callback) # VehicleState 설정 전
        rospy.Subscriber('/navigationData', NavigationData, self.navigation_data_callback)
        rospy.Subscriber('/detectionData', DetectionData, self.detection_data_callback)

    def can_output_cb(self, msg):
        self.vehicle_state.mode.data = mode_checker(msg.EPS_Control_Status.data, msg.ACC_Control_Status.data)  # off, on, steering, acc/brake
        self.vehicle_state.velocity.data = calc_wheel_velocity(msg.WHEEL_SPD_RR.data, msg.WHEEL_SPD_RL.data)  # m/s
        self.vehicle_state.gear.data = str(msg.G_SEL_DISP.data)    
        self.vehicle_state.signal.data = turn_signal_checker(msg.Turn_Left_En.data, msg.Turn_Right_En.data)  # blinker 같음
        self.ego_actuator.accel.data = float(msg.Long_ACCEL.data)
        self.ego_actuator.brake.data = float(msg.BRK_CYLINDER.data)
        self.ego_actuator.steer.data = float(msg.StrAng.data)

        self.speed_machine.velocity = self.vehicle_state.velocity.data
        self.trigger_events()


    def signal_cb(self, msg):
        self.system_status.systemSignal.data = int(msg.data)

    def user_input_cb(self, msg): #mode, signal, state, health
        mode = int(msg.user_mode.data)
        self.system_to_can(mode)
        self.system_status.systemMode.data = mode
        self.system_status.systemSignal.data = int(msg.user_signal.data)

    def nmea_sentence_cb(self, msg):
        parsed = sim_nmea_parser(msg.sentence)
        if parsed != None:
            if len(parsed) == 2:
                self.vehicle_state.position.x = parsed[0]
                self.vehicle_state.position.y = parsed[1]
            elif len(parsed) == 1:
                self.vehicle_state.heading.data = parsed[0]

    def nav_sat_fix_cb(self, msg):
        self.vehicle_state.header = msg.header
        if not self.check_error(self.vehicle_state.position.x, msg.latitude,30):
            self.vehicle_state.position.x = msg.latitude
        if not self.check_error(self.vehicle_state.position.y, msg.longitude,30):
            self.vehicle_state.position.y = msg.longitude
    
    def heading_cb(self, msg):
        yaw = match_heading(msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w)
        self.vehicle_state.heading.data = yaw    

    def target_actuator_cb(self, msg):
        steer = np.clip(msg.steer.data*12.9, -500, 500)
        self.can_input.EPS_Cmd.data = steer#msg.steer.data * 12.9 
        self.can_input.ACC_Cmd.data = msg.accel.data if msg.accel.data > 0 else -msg.brake.data
    
    def objects_cb(self, msg):
        self.detection_data = DetectionData()
        for obj in msg.poses:
            object_info = ObjectInfo()
            object_info.type.data = int(obj.position.z)
            object_info.position.x = float(obj.position.x)
            object_info.position.y = float(obj.position.y)
            object_info.velocity.data = float(obj.orientation.x)
            object_info.heading.data = float(obj.orientation.y)
            self.detection_data.objects.append(object_info)

    # def vehicle_state_callback(self, data):
    #     self.speed_machine.velocity = data.velocity.data
    #     self.trigger_events()

    def navigation_data_callback(self, data):
        self.speed_machine.road_type = data.road_type.data
        self.trigger_events()

    def detection_data_callback(self, data):
        self.speed_machine.front_obstacle = data.front_obstacle.data
        self.trigger_events()

    def trigger_events(self):
        if self.speed_machine.accel_on_straight():
            self.speed_machine.accel_event()
        if self.speed_machine.keep_on_straight():
            self.speed_machine.keep_event()
        if self.speed_machine.decel_on_straight():
            self.speed_machine.decel_event()
        if self.speed_machine.accel_on_curve():
            self.speed_machine.accel_event()
        if self.speed_machine.keep_on_curve():
            self.speed_machine.keep_event()
        if self.speed_machine.decel_on_curve():
            self.speed_machine.decel_event()
        if self.speed_machine.accel_on_bank():
            self.speed_machine.accel_event()
        if self.speed_machine.keep_on_bank():
            self.speed_machine.keep_event()
        if self.speed_machine.decel_on_bank():
            self.speed_machine.decel_event()
        if self.speed_machine.emergency():
            self.speed_machine.emergency_event()

    def publish_speedmachine(self, state_value, max_vel):
        msg = Int8MultiArray(data=[state_value, max_vel])
        self.speedmachine_pub.publish(msg)

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
        self.lane_data_pub.publish(self.lane_data)
        self.detection_data_pub.publish(self.detection_data)
        self.ego_actuator_pub.publish(self.ego_actuator)

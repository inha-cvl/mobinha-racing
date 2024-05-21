import rospy

from drive_msgs.msg import *
from geometry_msgs.msg import PoseArray
from nmea_msgs.msg import Sentence
from std_msgs.msg import Float32MultiArray

from libs.message_handler import *

class ROSHandler():
    def __init__(self):
        rospy.init_node('drive_message', anonymous=False)
        
        self.set_messages()
        self.set_publisher_protocol()
        self.set_subscriber_protocol()

    def set_messages(self):
        self.can_input = CANInput()
        self.can_input.EPS_Speed.data = 30
        self.sensor_data = SensorData()
        self.system_status = SystemStatus()
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
    
    def set_subscriber_protocol(self):
        rospy.Subscriber('/CANOutput', CANOutput, self.can_output_cb)
        rospy.Subscriber('/planning/local_action_set', PoseArray, self.local_action_set_cb)
        rospy.Subscriber('/nmea_sentence', Sentence, self.nmea_sentence_cb)
        rospy.Subscriber('/ui/user_input', Float32MultiArray, self.user_input_cb)
        rospy.Subscriber('/control/target_actuator', Actuator, self.target_actuator_cb)
        #SystemHealth : Float32MultiArray
        #Camera : image (?)

    def can_output_cb(self, msg):
        self.vehicle_state.mode.data = mode_checker(msg.EPS_Control_Status.data, msg.ACC_Control_Status.data)
        self.vehicle_state.velocity.data = calc_wheel_velocity(msg.WHEEL_SPD_RR.data, msg.WHEEL_SPD_RL.data)
        self.vehicle_state.gear.data = str(msg.G_SEL_DISP.data)
        self.vehicle_state.signal.data = turn_signal_checker(msg.Turn_Left_En.data, msg.Turn_Right_En.data)
        self.ego_actuator.accel.data = float(msg.Long_ACCEL.data)
        self.ego_actuator.brake.data = float(msg.BRK_CYLINDER.data)
        self.ego_actuator.steer.data = float(msg.StrAng.data)

    def signal_cb(self, msg):
        self.system_status.systemSignal.data = int(msg.data)

    def user_input_cb(self, msg): #mode, signal, state, health
        if len(msg.data) > 0:
            mode = int(msg.data[0])
            self.system_to_can(mode)
            self.system_status.systemMode.data = mode
            self.system_status.systemSignal.data = int(msg.data[1])

    def nmea_sentence_cb(self, msg):
        parsed = nmea_parser(msg.sentence)
        if len(parsed) == 2:
            self.vehicle_state.position.x = parsed[0]
            self.vehicle_state.position.y = parsed[1]
        elif len(parsed) == 1:
            self.vehicle_state.heading.data = parsed[0]    
    
    def target_actuator_cb(self, msg):
        self.can_input.EPS_Cmd.data = msg.steer.data * 12.9
        self.can_input.ACC_Cmd.data = msg.accel.data if msg.accel.data > 0 else -msg.brake.data

    def system_to_can(self, mode):
        #TODO: gear value checking
        if self.vehicle_state.mode.data == 0:
            self.can_input.EPS_En.data = 1 if mode in (1, 2) else 0
            self.can_input.ACC_En.data = 1 if mode in (1, 3) else 0

        elif mode == 0 and self.vehicle_state.mode.data >= 1:
            self.can_input.EPS_En.data = 0
            self.can_input.ACC_En.data = 0
        
    def local_action_set_cb(self, msg):
        print(msg)
    
    def publish(self):
        self.can_input_pub.publish(self.can_input)
        self.sensor_data_pub.publish(self.sensor_data)
        self.system_status_pub.publish(self.system_status)
        self.vehicle_state_pub.publish(self.vehicle_state)
        self.lane_data_pub.publish(self.lane_data)
        self.detection_data_pub.publish(self.detection_data)

        self.ego_actuator_pub.publish(self.ego_actuator)
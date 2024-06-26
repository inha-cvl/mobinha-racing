import rospy

from drive_msgs.msg import *
from std_msgs.msg import Float32MultiArray

STEER_RATIO = 12.9
MPS_TO_KPH = 3.6

class ROSHandler():
    def __init__(self):
        rospy.init_node('ui', anonymous=False)
        
        self.set_values()
        self.set_publisher_protocol()
        self.set_subscriber_protocol()

    def set_values(self):
        self.ego_value = {'velocity':0, 'steer': 0, 'accel': 0, 'brake': 0, 'gear':'P'}
        self.target_value = {'velocity':0, 'steer': 0, 'accel': 0, 'brake': 0}
        self.can_inform = {'eps_status':'Off', 'acc_status':'off'}
        self.system_status = {'mode': 0, 'signal':0, 'lap_count': 0}
        self.user_input = UserInput()
        self.user_value = {'user_mode': 0, 'user_signal': 0}
        self.lane_number = 0

    def set_publisher_protocol(self):
        self.pub_user_input = rospy.Publisher('/UserInput',UserInput, queue_size=1)
        
    def set_subscriber_protocol(self):
        rospy.Subscriber('/CANOutput', CANOutput, self.can_output_cb)
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/control/target_actuator', Actuator, self.target_actuator_cb)
        rospy.Subscriber('/NavigationData', NavigationData, self.navigation_data_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        rospy.Subscriber('/LaneData', LaneData, self.lane_data_cb)

    def can_output_cb(self, msg):
        self.ego_value['steer'] = -1*float(msg.StrAng.data)
        self.ego_value['accel'] = float(msg.Long_ACCEL.data)
        self.ego_value['brake'] = float(msg.BRK_CYLINDER.data)
        self.can_inform['eps_status'] = str(msg.EPS_En_Status.data)
        self.can_inform['acc_status'] = str(msg.ACC_En_Status.data)

    def vehicle_state_cb(self, msg):
        self.ego_value['velocity'] = int(msg.velocity.data*MPS_TO_KPH)
        self.ego_value['gear'] = str(msg.gear.data)

    def target_actuator_cb(self, msg):
        self.target_value['steer'] = -1*msg.steer.data
        self.target_value['accel'] = msg.accel.data
        self.target_value['brake'] = msg.brake.data
    
    def navigation_data_cb(self, msg):
        self.target_value['velocity'] = int(msg.plannedVelocity.data*MPS_TO_KPH)
    
    def system_status_cb(self, msg):
        self.system_status['mode'] = int(msg.systemMode.data)
        self.system_status['signal'] = int(msg.systemSignal.data)
        self.system_status['lap_count'] = msg.lapCount.data

    # lane_number
    def lane_data_cb(self, msg):
        if msg.currentLane.currentLane != 0:
            self.lane_number = msg.currentLane.currentLane
    
    def publish(self):
        self.user_input.user_mode.data = self.user_value['user_mode']
        self.user_input.user_signal.data = self.user_value['user_signal']
        self.pub_user_input.publish(self.user_input)
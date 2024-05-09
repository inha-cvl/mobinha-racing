import rospy

from drive_msgs.msg import *
from std_msgs.msg import Float32

class ROSHandler():
    def __init__(self):
        rospy.init_node('Control', anonymous=False)
        
        self.set_publisher_protocol()
        
        self.system_status = 0
        self.target_velocity = 0
        self.current_velocity = 0

        self.set_subscriber_protocol()

        self.target_actuator = Actuator()

    def set_publisher_protocol(self):
        self.target_actuator_pub = rospy.Publisher('/control/target_actuator', Actuator, queue_size=1)
    
    def set_subscriber_protocol(self):
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/test/target_velocity', Float32, self.target_velocity_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
    
    def system_status_cb(self, msg):
        self.system_status = msg.systemState.data 
    
    def target_velocity_cb(self, msg):
        self.target_velocity = msg.data
    
    def vehicle_state_cb(self, msg):
        self.current_velocity = msg.velocity.data
    
    def publish(self, acc):
        if acc > 0:
            accel = acc
            brake = 0
        else:
            accel = 0
            brake = -acc
        self.target_actuator.accel.data = accel
        self.target_actuator.brake.data = brake
        self.target_actuator_pub.publish(self.target_actuator)
import rospy
from drive_message.msg import *

class ROSHandler():
    def __init__(self, TH):
        rospy.init_node('Transmitter', anonymous=False)
        self.decode_handler = TH.decode_handler
        self.encode_handler = TH.encode_handler

        self.set_protocol()
        self.set_messages()

    def set_protocol(self):
        self.can_output_pub = rospy.Publisher('/transmitter/can_output', CANOutput, queue_size=1)
    
    def set_messages(self):
        self.can_output = CANOutput()
        for values in self.decode_handler.values():
            for key, value in values.items():
                getattr(self.can_output, key).data = value

        self.vehicle_state.mode.data = self.vehicle_state_dict['ACC_En_Status']['mode']
        self.vehicle_state.velocity.data = self.vehicle_state_dict['VS']['velocity']
        self.vehicle_state.heading.data = 0.0
        self.vehicle_state.position.x = 0
        self.vehicle_state.position.y = 0
        self.vehicle_state.position.z = 0

    def send_can_output(self):
        self.can_output_pub.publish(self.can_output)

    def update_can_output(self, message_dict):
        for key, value in message_dict.items():
            getattr(self.can_output, key).data= value
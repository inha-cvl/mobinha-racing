import rospy

from drive_message.msg import *



class ROSHandler():
    def __init__(self, TH):
        
        rospy.init_node('Transmitter', anonymous=False)
        self.decode_handler = TH.decode_handler
        self.encode_handler = TH.encode_handler

        self.set_protocol()
        self.set_messages()

        self.alive_cnt = -1

    def set_protocol(self):
        self.can_output_pub = rospy.Publisher('/transmitter/can_output', CANOutput, queue_size=1)
        rospy.Subscriber('/test/can_input', CANInput, self.can_input_cb)


    def set_messages(self):
        self.can_output = CANOutput()
        for values in self.decode_handler.values():
            for key, value in values.items():
                getattr(self.can_output, key).data = value

        self.can_input = CANInput()
        for values in self.encode_handler.values():
            for key, value in values.items():
                getattr(self.can_input, key).data = value
        
    def update_can_output(self, message_dict):
        for key, value in message_dict.items():
            getattr(self.can_output, key).data = str(value)
    
    def update_can_inputs(self):
        self.update_alive_cnt()
        dicts = []
        for values in self.encode_handler.values():
            for key in values.keys():
                if key == 'Aliv_Cnt':
                    values[key] = self.alive_cnt
                else:
                    values[key]=getattr(self.can_input, key).data
            dicts.append(values)
        return dicts

    def update_alive_cnt(self):
        self.alive_cnt += 1
        if self.alive_cnt >= 256:
            self.alive_cnt = 0

    def send_can_output(self):
        self.can_output_pub.publish(self.can_output)

    def can_input_cb(self, msg):
        self.can_input = msg
#!/usr/bin/env python
import rospy
import asyncio
import sys
import signal
from pprint import pprint

from drive_message.msg import CANInput

###
from drive_message.msg import CANOutput

from transmitter_handler import TransmitterHandler

from apid_origin import Apid

def signal_handler(sig, frame):
    sys.exit(0)


class CANInputTest:
    def __init__(self):
        rospy.init_node("can_input_test")
        self.TH = TransmitterHandler()
        self.set_datum()
        self.set_protocol()
        self.set_messages()
        self.set_controller()
        
    def set_controller(self):
        self.Apid = Apid()
        self.current_v = 0
        self.target_v = 0

    def set_protocol(self):
        self.can_input_pub = rospy.Publisher('/test/can_input', CANInput, queue_size=1)
        self.can_output_sub = rospy.Subscriber('/transmitter/can_output', CANOutput, self.can_output_cb)

    def set_datum(self):
        self.can_input = CANInput()
        self.can_input_dict = {
            'EPS_En':0,
            'EPS_Override_Ignore':0,
            'EPS_Speed':0,
            'ACC_En':0,
            'AEB_En':0,
            'Turn_Signal':0,
            'AEB_decel_value':0,
            'EPS_Cmd':0,
            'ACC_Cmd':0
        }

        for values in self.TH.encode_handler.values():
            for key, value in values.items():
                getattr(self.can_input, key).data = value
                self.can_input_dict[key] = value

    def publish(self):
        self.can_input_pub.publish(self.can_input)

    async def ros_publisher(self):
        try:
            while not rospy.is_shutdown():
                await asyncio.get_event_loop().run_in_executor(None, self.publish)
                await asyncio.sleep(0.05) #20hz
        except Exception as e:
            rospy.logerr(f"Error in ros_publisher: {e}")
    
    def update_value(self):
        ## controller update 
        output = self.Apid.run(self.current_v, self.target_v)
        # output mapping : output -> brake, accel
        if output < 0:
            output (3 / 100) * output
        else:
            output (1 / 100) * output
        self.can_input_dict['ACC_Cmd'] = output
        for key, value in self.can_input_dict.items():
            print(key, value)
            getattr(self.can_input, key).data = value
        

    def get_user_value(self):
        pprint(self.can_input_dict)
        print('Index: EPS_En EPS_Override_Ignore EPS_Speed ACC_En AEB_En Turn_Signal AEB_decel_value EPS_Cmd ACC_Cmd')
        print('       [ 0  ] [        1        ] [   2   ] [  3 ] [ 4  ] [   5     ] [     6       ] [  7  ] [  8  ]')
        print('Over to Enter 99')
        keys = list(self.can_input_dict.keys())
        while True:
            _in = input('Index Value: ').split()
            if int(_in[0]) == 99:
                break
            key = keys[int(_in[0])]
            if int(_in[0]) in [2,6, 8,9]:
                value = float(_in[1])
            else:
                value = int(_in[1])
            self.can_input_dict[key] = value

    async def get_user(self):
        try:
            while not rospy.is_shutdown():
                await asyncio.get_event_loop().run_in_executor(None, self.get_user_value)
                self.update_value()
                await asyncio.sleep(0.1)

        except Exception as e:
            rospy.logerr(f"Error in get_user: {e}")

    def run(self):
        loop = asyncio.get_event_loop()
        pub_task = loop.create_task(self.ros_publisher())
        get_task = loop.create_task(self.get_user())
        loop.run_forever()
    
    def set_messages(self):
        self.can_output = CANOutput()
        for values in self.decode_handler.values():
            for key, value in values.items():
                getattr(self.can_output, key).data = value

    def can_output_cb(self, msg): # self.TH 안에 저장
        for keys, values in self.TH.decode_handler.items():
            for key in values.keys():
                values[key] = getattr(msg, key).data
    
    def get_current_v(self):
        vRR = self.TH.decode_handler[0x712]['WHEEL_SPD_RR']
        vRL = self.TH.decode_handler[0x712]['WHEEL_SPD_RL']
        self.current_v = (vRR + vRL)/7.2
    
    def update_target_v(self):
        ## manual
        ## case 1 : constant
        self.target_v = 20 / 3.6 # km/h

        ## case 2 : sinusoidal
        # amplitude = 5
        # offset = 20
        # number = datetime.datetime.now().microsecond%1000000
        # while number >= 10:
        #     number //= 10
        # self.target_v = offset + amplitude * np.sin(number*2*np.pi/9) / 3.6

        ## case 3 : step
        # amplitude = 5
        # offset = 20
        # number = datetime.datetime.now().second%10
        # if number < 5:
        #     step = 1
        # else:
        #     step = -1
        # self.target_v = offset + amplitude*step / 3.6

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    can_input_test = CANInputTest()
    can_input_test.run()
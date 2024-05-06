#!/usr/bin/env python
import rospy
import asyncio
import sys
import signal
from pprint import pprint

from drive_message.msg import CANInput

from transmitter_handler import TransmitterHandler

def signal_handler(sig, frame):
    sys.exit(0)


class CANInputTest:
    def __init__(self):
        rospy.init_node("can_input_test")
        self.TH = TransmitterHandler()
        self.set_datum()
        self.set_protocol()
        
    def set_protocol(self):
        self.can_input_pub = rospy.Publisher('/test/can_input', CANInput, queue_size=1)

    def set_datum(self):
        self.can_input = CANInput()
        self.can_input_dict = { # 출력을 위한 dict
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

        for values in self.TH.encode_handler.values(): # self.EAIT_Control_01, self.EAIT_Control_02
            for key, value in values.items():
                getattr(self.can_input, key).data = value # self.can_input 메시지 업데이트
                self.can_input_dict[key] = value # self.can_input_dict 업데이트

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
        pub_task = loop.create_task(self.ros_publisher()) # garbage collector에 의해 수거되지 않도록 변수에 저장
        get_task = loop.create_task(self.get_user())
        loop.run_forever()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    can_input_test = CANInputTest()
    can_input_test.run()
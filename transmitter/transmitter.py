#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
import asyncio
import can
import rospy
from pprint import pprint

import sys
import signal

from transmitter_handler import TransmitterHandler
from ros_handler import ROSHandler
import random

def signal_handler(sig, frame):
    sys.exit(0)

class Transmitter():
    def __init__(self):
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitrate=500000)
        self.TH = TransmitterHandler()
        self.RH = ROSHandler(self.TH)

    async def read_from_can(self):
        try:
            while not rospy.is_shutdown():
                message = await asyncio.get_event_loop().run_in_executor(None, self.bus.recv, 0.2)
                
                if message.arbitration_id == 1808: # EPS
                    message_dict = self.TH.decode_message(message)
                    print(message_dict['Override_Status'], message_dict['StrAng'])
                # if message.arbitration_id == 1809: # ACC
                #     message_dict = self.TH.decode_message(message)
                #     print(message_dict['ACC_Alive_Cnt'])

                if message:
                    message_dict = self.TH.decode_message(message)
                    if message_dict != None:
                        self.RH.update_can_output(message_dict)
                # await asyncio.sleep(0.002)  # 500Hz, 2ms 간격
        except Exception as e:
            rospy.logerr(f"Error in read_from_can: {e}")
        finally:
            self.bus.shutdown()

    async def send_on_can(self):
        try:
            while not rospy.is_shutdown():
                dicts = self.RH.update_can_inputs()
                can_messages = self.TH.encode_message(dicts)
                for can_message in can_messages:
                    await asyncio.get_event_loop().run_in_executor(None, self.bus.send, can_message)
                    # print(can_message.arbitration_id)
                    # if can_message.arbitration_id == 343:
                    #     # can_message.data[3] = 0x01
                    #     print(can_message)
                    await asyncio.sleep(0.01)  # 100Hz, 10ms 간격
        except Exception as e:
            rospy.logerr(f"Error in send_on_can: {e}")
    
    async def ros_publisher(self):
        try:
            await asyncio.sleep(0.5)
            while not rospy.is_shutdown():
                await asyncio.get_event_loop().run_in_executor(None, self.RH.send_can_output)
                await asyncio.sleep(0.05)  # 50Hz, 0ms 간격
        except Exception as e:
            rospy.logerr(f"Error in ros_publisher: {e}")
        
    def run(self):
        loop = asyncio.get_event_loop()
        read_task = loop.create_task(self.read_from_can())
        send_task = loop.create_task(self.send_on_can())
        pub_task = loop.create_task(self.ros_publisher())
        loop.run_forever()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    transmitter = Transmitter()
    transmitter.run()

if __name__ == "__main__":
    main()

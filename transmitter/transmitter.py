#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
import asyncio
import can
import rospy
from pprint import pprint

from transmitter_handler import TransmitterHandler
from ros_handler import ROSHandler
import random

class Transmitter():
    def __init__(self):
        #self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitrate=500000)
        self.TH = TransmitterHandler()
        self.RH = ROSHandler(self.TH)

    async def read_from_can(self):
        try:
            while not rospy.is_shutdown():
                message = await asyncio.get_event_loop().run_in_executor(None, self.bus.recv, 0.2)
                if message:
                    message_dict = self.TH.decode_message(message)
                    if message_dict != None:
                        self.RH.update_can_output(message_dict)
                await asyncio.sleep(0.002)  # 500Hz, 2ms 간격
        except Exception as e:
            rospy.logerr(f"Error in read_from_can: {e}")
        finally:
            self.bus.shutdown()

    async def send_on_can(self):
        try:
            while not rospy.is_shutdown():

                data = {"SomeSignal": 123}  # 예제 신호 및 값
                message = self.dbc.encode_message('SomeMessage', data)

                can_message = can.Message(arbitration_id=message.frame_id, data=message.data, is_extended_id=message.is_extended_frame)
                await asyncio.get_event_loop().run_in_executor(None, self.bus.send, can_message)
                
                await asyncio.sleep(0.01)  # 100Hz, 10ms 간격
        except Exception as e:
            rospy.logerr(f"Error in send_on_can: {e}")
    
    async def publisher(self):
        try:
            while not rospy.is_shutdown():
                await asyncio.get_event_loop().run_in_executor(None, self.RH.send_can_output)
                await asyncio.sleep(0.02)  # 50Hz, 10ms 간격
        except Exception as e:
            rospy.logerr(f"Error in publisehr: {e}")
        
    def run(self):
        loop = asyncio.get_event_loop()
        #read_task = loop.create_task(self.read_from_can())
        #send_task = loop.create_task(self.send_on_can())
        pub_task = loop.create_task(self.publisher())
        loop.run_forever()

def main():

    transmitter = Transmitter()
    transmitter.run()

if __name__ == "__main__":
    main()

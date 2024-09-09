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
        self.bus1 = can.ThreadSafeBus(interface='socketcan', channel='can0', bitrate=500000)
        self.bus2 = can.ThreadSafeBus(interface='socketcan', fd=True, channel='can2', bitrate=500000)
        self.TH = TransmitterHandler()
        self.RH = ROSHandler(self.TH)

    async def read_from_can1(self):
        try:
            while not rospy.is_shutdown():
                message = await asyncio.get_event_loop().run_in_executor(None, self.bus1.recv, 0.2)               
                if message:
                    message_dict = self.TH.decode_message1(message)
                    if message_dict != None:
                        self.RH.update_can_output(message_dict)
                # await asyncio.sleep(0.002)  # 500Hz, 2ms 간격
        except Exception as e:
            rospy.logerr(f"Error in read_from_can1: {e}")
        finally:
            self.bus1.shutdown()

    async def send_on_can1(self):
        try:
            while not rospy.is_shutdown():
                dicts = self.RH.update_can_inputs()
                can_messages = self.TH.encode_message(dicts)
                for can_message in can_messages:
                    await asyncio.get_event_loop().run_in_executor(None, self.bus1.send, can_message)
                    await asyncio.sleep(0.02)  # 100Hz, 10ms 간격
        except Exception as e:
            rospy.logerr(f"Error in send_on_can1: {e}")
    
    async def read_from_can2(self):
        try:
            while not rospy.is_shutdown():
                message = await asyncio.get_event_loop().run_in_executor(None, self.bus2.recv, 0.2)     
                if message:
                    type, message_dict = self.TH.decode_message2(message)
                    if message_dict is not None:
                        self.RH.update_radar_output(type, message_dict)
                # await asyncio.sleep(0.002)  # 500Hz, 2ms 간격
        except Exception as e:
            rospy.logerr(f"Error in read_from_can2: {e}")
        finally:
            self.bus2.shutdown()
    
    async def ros_publisher(self):
        try:
            await asyncio.sleep(0.5)
            cnt = 0 
            while not rospy.is_shutdown():
                await asyncio.get_event_loop().run_in_executor(None, self.RH.send_can_output)
                await asyncio.get_event_loop().run_in_executor(None, self.RH.send_radar_output, cnt)
                cnt += 1
                await asyncio.sleep(0.05)  # 20Hz, 0ms 간격
        except Exception as e:
            rospy.logerr(f"Error in ros_publisher: {e}")
        
    def run(self):
        loop = asyncio.get_event_loop()
        read_task1 = loop.create_task(self.read_from_can1())
        send_task1 = loop.create_task(self.send_on_can1())
        read_task2 = loop.create_task(self.read_from_can2())
        pub_task = loop.create_task(self.ros_publisher())
        loop.run_forever()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    transmitter = Transmitter()
    transmitter.run()

if __name__ == "__main__":
    main()

#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
import asyncio
import can
import rospy

import sys
import signal

from transmitter_handler import TransmitterHandler

def signal_handler(sig, frame):
    sys.exit(0)

class CAN2Test():
    def __init__(self):
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can1', bitrate=500000)
        self.TH = TransmitterHandler()

    async def read_from_can(self):
        try:
            while not rospy.is_shutdown():
                message = await asyncio.get_event_loop().run_in_executor(None, self.bus.recv, 0.2)               
                if message:
                    print(message)
        except Exception as e:
            rospy.logerr(f"Error in read_from_can: {e}")
        finally:
            self.bus.shutdown()
        
    def run(self):
        loop = asyncio.get_event_loop()
        read_task = loop.create_task(self.read_from_can())
        loop.run_forever()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    can2_test = CAN2Test()
    can2_test.run()

if __name__ == "__main__":
    main()

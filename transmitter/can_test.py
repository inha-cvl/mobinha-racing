#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
# sudo ip link set can0 type can bitrate 500000 sample-point 0.750 dbitrate 2000000 dsample-point 0.750 fd on
import asyncio
import can
import cantools.database
import rospy

import sys
import os
import signal
import cantools

from transmitter_handler import TransmitterHandler

def signal_handler(sig, frame):
    sys.exit(0)

class CANTest():
    def __init__(self):
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can1', bitrate=500000)
        self.TH = TransmitterHandler()
        self.dbc = cantools.database.load_file('./hyundai_ccan.dbc')

    async def read_from_can(self):
        try:
            while not rospy.is_shutdown():
                message = await asyncio.get_event_loop().run_in_executor(None, self.bus.recv, 0.2)               
                if message:
                    try:
                       
                        if message.arbitration_id == 1419:
                            print(hex(message.arbitration_id))
                            print(self.dbc.decode_message(message.arbitration_id, message.data))
                            print('\n')
                    except Exception as E:
                        pass
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
    can2_test = CANTest()
    can2_test.run()

if __name__ == "__main__":
    main()

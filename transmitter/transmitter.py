#!/usr/bin/python3
# sudo ip link set can0 up type can bitrate 500000
import sys
import asyncio
import can
import rospy

from PyQt5.QtWidgets import QApplication

from transmitter_handler import TransmitterHandler
from message_viewer import MessageViewer

class Transmitter():
    def __init__(self):
        self.bus = can.ThreadSafeBus(interface='socketcan', channel='can0', bitrate=500000)
        self.TH = TransmitterHandler()
        self.MV = MessageViewer(self.TH) 

    async def read_from_can(self):
        try:
            while not rospy.is_shutdown():
                message = await asyncio.get_event_loop().run_in_executor(None, self.bus.recv, 0.2)
                if message:
                    message_dict = self.TH.decode_message(message)
                    self.MV.update_table(message.arbitration_id, message_dict)
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

    def run(self):
        loop = asyncio.get_event_loop()
        read_task = loop.create_task(self.read_from_can())
        send_task = loop.create_task(self.send_on_can())
        loop.run_forever()

def main():
    rospy.init_node('Transmitter', anonymous=False)
    app = QApplication(sys.argv)
    transmitter = Transmitter()
    transmitter.run()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

import rospy
import csv
import signal
import sys
from drive_msgs.msg import VehicleState  # your_package를 실제 패키지 이름으로 변경

class VehicleStateRecorder:
    def __init__(self):
        self.csv_file = open('./csv/0620.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['latitude', 'longitude'])
        rospy.init_node('vehicle_state_recorder', anonymous=True)
        rospy.Subscriber('/VehicleState', VehicleState, self.callback)  # vehicle_state_topic을 실제 토픽 이름으로 변경
        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.spin()

    def callback(self, data):
        position_x = data.position.x
        position_y = data.position.y
        self.csv_writer.writerow([position_x, position_y])

    def signal_handler(self, sig, frame):
        print('Exiting...')
        self.csv_file.close()
        sys.exit(0)

if __name__ == '__main__':
    recorder = VehicleStateRecorder()

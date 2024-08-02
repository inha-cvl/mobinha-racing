import rospy
import sys
import signal
import numpy as np
from filterpy.kalman import KalmanFilter

from drive_msgs.msg import CANOutput
from ublox_msgs.msg import NavPVT, NavATT
from sensor_msgs.msg import NavSatFix

from ros_handler import ROSHandler
from dr_bicycle import DR_BICYCLE
from imu_heading import ImuHeading

def signal_handler(sig, frame):
    sys.exit(0)

class KFLocalization:
    def __init__(self):
        self.initailize = True
        self.RH = ROSHandler(map)
        self.DR = DR_BICYCLE()
        self.IH = ImuHeading()

        self.nav_pos = self.RH.nav_pos
        self.nav_heading = self.RH.navatt.heading
        self.dr_pos = self.DR.dr_pos
        self.dr_heading = self.DR.dr_heading
        self.imu_heading = self.IH.imu_heading
        
        self.lidar_pos = None  #TODO
        self.lidar_heading = None  #TODO
        
        self.kf_heading = self.init_heading_kalman_filter()
        self.kf_position = self.init_position_kalman_filter()

        self.filtered_heading = None
        self.filtered_position = None

    def init_heading_kalman_filter(self):
        kf = KalmanFilter(dim_x=2, dim_z=1)
        kf.x = np.zeros(2)
        kf.P = np.eye(2) * 1000.
        kf.R = np.eye(1) * 5.
        kf.Q = np.eye(2)
        kf.F = np.eye(2)
        kf.H = np.array([[1, 0]])
        return kf

    def init_position_kalman_filter(self):
        kf = KalmanFilter(dim_x=4, dim_z=2)
        kf.x = np.zeros(4)
        kf.P = np.eye(4) * 1000.
        kf.R = np.eye(2) * 5.
        kf.Q = np.eye(4)
        kf.F = np.eye(4)
        kf.F[0, 2] = kf.F[1, 3] = 1
        kf.H = np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0]])
        return kf

    def update_sensors(self):
        self.nav_pos = self.RH.nav_pos
        self.nav_heading = self.RH.navatt.heading if self.RH.navatt else None
        self.dr_pos = self.DR.dr_pos
        self.dr_heading = self.DR.dr_heading
        self.imu_heading = self.IH.imu_heading
        
        # TODO: Update self.lidar_pos and self.lidar_heading with Lidar data
    
    def calculate_diffs(self):
        self.heading_diff = self.nav_heading - self.kf_heading
        self.position_diff = np.linalg.norm(np.array(self.nav_pos) - np.array(self.kf_position))
        print(f"Heading diff: {self.heading_diff}")
        print(f"Position diff: {self.position_diff}")

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.update_sensors()

            if self.nav_heading is not None:
                self.kf_heading.predict()
                self.kf_heading.update(self.nav_heading)
                self.filtered_heading = self.kf_heading.x[0]
                # rospy.loginfo(f"Filtered Heading: {filtered_heading}")

            if self.nav_pos is not None:
                self.kf_position.predict()
                self.kf_position.update(np.array([self.nav_pos[0], self.nav_pos[1]]))
                self.filtered_heading = self.kf_position.x[:2]
                # rospy.loginfo(f"Filtered Position: {filtered_position}")
            
            if None not in [self.filtered_heading, self.filtered_position]:
                self.RH.publish(self.filtered_heading, self.filtered_position)
                self.calculate_diffs()
            
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    kf_localization = KFLocalization()
    kf_localization.run()

if __name__ == "__main__":
    main()
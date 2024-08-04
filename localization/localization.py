import rospy
import sys
import signal
import numpy as np
from filterpy.kalman import KalmanFilter

from drive_msgs.msg import CANOutput
from ublox_msgs.msg import NavPVT, NavATT
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

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

        self.nav_heading = None
        self.nav_pos = None
        self.dr_heading = None
        self.dr_pos = [None, None]
        self.imu_heading = None
        
        self.nav_heading_last = None
        self.nav_pos_last = None
        self.dr_heading_last = None
        self.dr_pos_last = None
        self.imu_heading_last = None
        
        self.lidar_heading = None  #TODO
        self.lidar_pos = None  #TODO
        
        self.kf_heading = self.init_heading_kalman_filter()
        self.kf_pos = self.init_pos_kalman_filter()

        self.filtered_heading = None
        self.filtered_pos = [None, None]

        self.nav_cw_cnt = 0
        self.imu_cw_cnt = 0
        self.dr_cw_cnt = 0
        self.p_nav_heading = None
        self.p_imu_heading = None
        self.p_dr_heading = None

        self.update_sensors()
        
    def update_sensors(self):
        self.nav_heading_last = self.nav_heading
        self.nav_pos_last = self.nav_pos
        self.dr_heading_last = self.dr_heading
        self.dr_pos_last = self.dr_pos
        self.imu_heading_last = self.imu_heading

        self.nav_pos = self.RH.nav_pos
        self.nav_heading = self.RH.nav_heading

        #tmp
        self.imu_heading = self.IH.imu_corr_heading
        self.dr_heading = self.DR.dr_heading
        self.dr_pos = self.DR.dr_pos

        # TODO: Update self.lidar_pos and self.lidar_heading with Lidar data


    def valid_pos(self, pos_last, pos_now, hz): # not used yet
        # threshold : under 120[km/h]
        diff = ((pos_now[0]-pos_last[0])**2 + (pos_now[1]-pos_last[1])**2)**0.5
        result = diff*hz < 120/3.6
        
        return result

    def valid_hdg(self, hdg_last, hdg_now, hz): # not used yet
        # threshold : under 100[deg/s]
        val = abs(hdg_last - hdg_now)
        diff = min(val, 360 - val)
        result = diff*hz < 5
        
        return result
        
    def heading_postprocess(self):
        if self.nav_heading - self.nav_heading_last < -355:
            self.nav_cw_cnt -= 1
        if self.nav_heading - self.nav_heading_last > 355:
            self.nav_cw_cnt += 1
        self.p_nav_heading = self.nav_heading - self.cw_cnt * 360

        if self.imu_heading - self.imu_heading_last < -355:
            self.imu_cw_cnt -= 1
        if self.imu_heading - self.imu_heading_last > 355:
            self.imu_cw_cnt += 1
        self.p_imu_heading = self.imu_heading - self.cw_cnt * 360

        if self.dr_heading - self.dr_heading_last < -355:
            self.dr_cw_cnt -= 1
        if self.dr_heading - self.dr_heading_last > 355:
            self.dr_cw_cnt += 1
        self.p_dr_heading = self.dr_heading - self.cw_cnt * 360

    def init_heading_kalman_filter(self):
        kf = KalmanFilter(dim_x=1, dim_z=3)  # dim: state vector=1, measurement vector=3
        kf.x = np.zeros(1)  # initial heading value
        kf.P = np.eye(1) * 1000.  # initial cov matrix
        kf.R = np.diag([1, 1, 1])  # initial meas_cov matrix
        kf.Q = np.eye(1) * 0.01  # inital process noise matrix
        kf.F = np.eye(1)  # state transition matrix
        kf.H = np.array([[1], [1], [1]])
        return kf

    def init_pos_kalman_filter(self):
        kf = KalmanFilter(dim_x=2, dim_z=4)
        kf.x = np.zeros(2)
        kf.P = np.eye(2) * 1000.
        kf.R = np.diag([1, 1, 1, 1])
        kf.Q = np.eye(2) * 0.01
        kf.F = np.eye(2)
        kf.F = np.eye(2) 
        kf.H = np.array([
        [1, 0],  # nav_pos x
        [0, 1],  # nav_pos y
        [1, 0],  # dr_pos x
        [0, 1]   # dr_pos y
    ])
        return kf
    
    def calculate_diffs(self):
        self.heading_diff = self.nav_heading - self.filtered_heading
        self.pos_diff = np.linalg.norm(np.array(self.nav_pos) - np.array(self.filtered_pos))
        # print(f"Heading diff: {self.heading_diff}")
        # print(f"pos diff: {self.pos_diff}")

    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.DR.run()
            self.IH.run()

            self.update_sensors()
            if None not in [self.nav_heading_last, self.imu_heading_last, self.dr_heading_last]:
                self.heading_postprocess()

            if self.nav_heading is not None and self.imu_heading is not None and self.dr_heading is not None:
                print("all headings are not None")
                self.kf_heading.predict()
                z = np.array([self.nav_heading, self.imu_heading, self.dr_heading])
                print("trying to update with", self.nav_heading)
                self.kf_heading.update(z)
                self.filtered_heading = self.kf_heading.x
                # print(self.filtered_heading)

            if self.nav_pos is not None and self.dr_pos is not None:
                self.kf_pos.predict()
                z = np.array([
                    self.nav_pos[0], self.nav_pos[1],
                    self.dr_pos[0], self.dr_pos[1]
                ])
                self.kf_pos.update(z)
                self.filtered_pos = self.kf_pos.x
                # print(self.filtered_pos)
            
            if None not in [self.filtered_heading, self.filtered_pos[0]]:
                self.RH.publish(self.filtered_heading, self.filtered_pos)
                self.calculate_diffs()
            
            print("Nav heading:", self.nav_heading)
            print("IMU heading:", self.imu_heading)
            print("DR heading:", self.dr_heading)
            print("Filtered heading:", self.filtered_heading)
            print("Nav pos:", self.nav_pos)
            print("DR pos:", self.dr_pos)
            print("Filtered pos:", self.filtered_pos)
            print("----------------------")
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    kf_localization = KFLocalization()
    kf_localization.run()

if __name__ == "__main__":
    main()

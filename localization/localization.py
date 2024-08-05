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

        self.nav_pos = None
        self.nav_heading = None
        self.dr_pos = [None, None]
        self.dr_heading = None
        self.imu_heading = None
        
        self.nav_pos_last = None
        self.nav_heading_last = None
        self.dr_pos_last = None
        self.dr_heading_last = None
        self.imu_heading_last = None
        
        self.lidar_pos = None  #TODO
        self.lidar_heading = None  #TODO
        
        self.kf_heading = self.init_heading_kalman_filter()
        self.kf_position = self.init_position_kalman_filter()

        self.filtered_heading = None
        self.filtered_position = [None, None]

        rospy.Subscriber('/dr/heading', Float32, self.dr_heading_cb)
        rospy.Subscriber('/dr/pos', Float32, self.dr_pos_cb)
        rospy.Subscriber('/imu/heading', Float32, self.imu_heading_cb)

        self.update_sensors()

    def dr_heading_cb(self, msg):
        self.dr_heading = msg.data
    
    def dr_pos_cb(self, msg):
        self.dr_pos = msg.data
    
    def imu_heading_cb(self, msg):
        self.imu_heading = msg.data
        
    def update_sensors(self):
        self.nav_pos_last = self.nav_pos
        self.nav_heading_last = self.nav_heading
        self.dr_pos_last = self.dr_pos
        self.dr_heading_last = self.dr_heading
        self.imu_heading_last = self.imu_heading

        self.nav_pos = self.RH.nav_pos
        self.nav_heading = self.RH.navatt.heading if self.RH.navatt else None

        #tmp
        self.imu_heading = self.IH.imu_heading
        self.dr_heading = self.DR.dr_heading
        self.dr_pos = self.DR.dr_pos

        # TODO: Update self.lidar_pos and self.lidar_heading with Lidar data


    def valid_position(self, pos_last, pos_now, hz): # not used yet
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
        kf = KalmanFilter(dim_x=2, dim_z=3)
        kf.x = np.zeros(2)
        kf.P = np.eye(2) * 1000.
        kf.R = np.diag([1, 10, 10])
        kf.Q = np.eye(2) * 0.01
        kf.F = np.eye(2)
        kf.H = np.array([[1, 0], [1, 0], [1, 0]])
        return kf

    def init_position_kalman_filter(self):
        kf = KalmanFilter(dim_x=4, dim_z=3)
        kf.x = np.zeros(4)
        kf.P = np.eye(4) * 1000.
        kf.R = np.diag([1, 10, 10])
        kf.Q = np.eye(4) * 0.01
        kf.F = np.eye(4)
        kf.F[0, 2] = kf.F[1, 3] = 1
        kf.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [1, 0, 0, 0]])
        return kf
    
    def calculate_diffs(self):
        self.heading_diff = self.nav_heading - self.filtered_heading
        self.position_diff = np.linalg.norm(np.array(self.nav_pos) - np.array(self.filtered_position))
        # print(f"Heading diff: {self.heading_diff}")
        # print(f"Position diff: {self.position_diff}")

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
                z = np.array([self.nav_heading, self.imu_heading, self.dr_heading]).reshape((3, 1))
                print("trying to update with", self.nav_heading)
                self.kf_heading.update(z)
                self.filtered_heading = self.kf_heading.x[0]
                # print(self.filtered_heading)
                # rospy.loginfo(f"Filtered Heading: {filtered_heading}")

            if self.nav_pos is not None and self.dr_pos is not None:
                self.kf_position.predict()
                z = np.array([self.nav_pos[0], self.nav_pos[1], self.dr_pos[0], self.dr_pos[1]]).reshape((4, 1))
                self.kf_position.update(z)
                self.filtered_position = self.kf_position.x[:2]
                # print(self.filtered_position)
                # rospy.loginfo(f"Filtered Position: {filtered_position}")
            
            if None not in [self.filtered_heading, self.filtered_position[0]]:
            # if self.filtered_heading is not None and self.filtered_position is not None:
                # print(self.filtered_heading, self.filtered_position)
                self.RH.publish(self.filtered_heading, self.filtered_position)
                self.calculate_diffs()
            
            # print("Nav heading:", self.nav_heading)
            # print("IMU heading:", self.imu_heading)
            # print("DR heading:", self.dr_heading)
            # print("Filtered heading:", self.filtered_heading)
            # print("Nav position:", self.nav_pos)
            # print("DR position:", self.dr_pos)
            # print("Filtered position:", self.filtered_position)
            # print("----------------------")
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    kf_localization = KFLocalization()
    kf_localization.run()

if __name__ == "__main__":
    main()

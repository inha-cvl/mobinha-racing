import numpy as np

from ahrs.filters import Madgwick

class ImuHeading():
    def __init__(self, RH):
        self.RH = RH
        self.madgwick = Madgwick()
        self.initial_offset = 0
        self.curve_list = self.RH.curve_list

        self.imu_corr_hdg = None

        self.last_s = None
        self.last_ns = None

        self.initial = True

        self.q = None

    def euler_to_quaternion(self, roll, pitch, hdg):
        cr = np.cos(roll / 2)
        sr = np.sin(roll / 2)
        cp = np.cos(pitch / 2)
        sp = np.sin(pitch / 2)
        cy = np.cos(hdg / 2)
        sy = np.sin(hdg / 2)

        q0 = cr * cp * cy + sr * sp * sy
        q1 = sr * cp * cy - cr * sp * sy
        q2 = cr * sp * cy + sr * cp * sy
        q3 = cr * cp * sy - sr * sp * cy

        return np.array([q0, q1, q2, q3])
    
    def check_msg_valid(self):
        nav_att_checklist = [self.RH.nav_roll, self.RH.nav_pitch, self.RH.nav_hdg]
        imu_checklist = [self.RH.imu_header, self.RH.imu_linear_acceleration, self.RH.imu_angular_velocity]

        nav_att_check = None not in nav_att_checklist
        imu_check = None not in imu_checklist

        checklist = [nav_att_check, imu_check]
        result = False not in checklist

        return result

    def run(self, best_hdg, nav_hdg_valid):
        if self.check_msg_valid():
            if self.initial:
                self.q = self.euler_to_quaternion(np.deg2rad(self.RH.nav_roll), np.deg2rad(self.RH.nav_pitch), np.deg2rad(self.RH.nav_hdg))
                self.last_s = self.RH.imu_header.stamp.secs
                self.last_ns = self.RH.imu_header.stamp.nsecs
                self.initial = False
            
            # hdg from imu
            accel = np.array([self.RH.imu_linear_acceleration.x, self.RH.imu_linear_acceleration.y, self.RH.imu_linear_acceleration.z])
            gyro = np.array([self.RH.imu_angular_velocity.x, self.RH.imu_angular_velocity.y, self.RH.imu_angular_velocity.z])
            
            # calculate dt
            ds = self.RH.imu_header.stamp.secs - self.last_s
            dns = self.RH.imu_header.stamp.nsecs - self.last_ns
            dt = ds + dns*1e-9
            self.last_s = self.RH.imu_header.stamp.secs
            self.last_ns = self.RH.imu_header.stamp.nsecs

            if not nav_hdg_valid: # wrong hdg
                self.q = self.madgwick.updateIMU(q=self.q, gyr=gyro, acc=accel, dt=dt)
                hdg = -np.rad2deg(np.arctan2(2.0*(self.q[0]*self.q[3] + self.q[1]*self.q[2]), 1.0 - 2.0*(self.q[2]**2 + self.q[3]**2)))
            else: # right hdg
                self.q = self.euler_to_quaternion(np.deg2rad(self.RH.nav_roll), np.deg2rad(self.RH.nav_pitch), np.deg2rad(best_hdg))
                self.cnt = 0
                hdg = -np.rad2deg(np.arctan2(2.0*(self.q[0]*self.q[3] + self.q[1]*self.q[2]), 1.0 - 2.0*(self.q[2]**2 + self.q[3]**2)))
                self.initial_offset = best_hdg - hdg


            # straight : -0.023
            # curve : -0.015
            if self.RH.curr_lane_id in self.curve_list:
                constant_offset = -0.010*self.cnt #turned right
            else:
                constant_offset = -0.0245*self.cnt

            hdg += self.initial_offset
            offseted_hdg = hdg + constant_offset

            # val = 0
            # if offseted_hdg < 0:
            #     val = 360
            # if offseted_hdg > 360:
            #     val = -360
            
            # clipped_hdg = -(offseted_hdg + val - 90)%360
            
            self.cnt += 1

            self.imu_corr_hdg = offseted_hdg
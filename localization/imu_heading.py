import rospy

import sys
import signal
import numpy as np

from ros_handler import ROSHandler
from ahrs.filters import Madgwick

def signal_handler(sig, frame):
    sys.exit(0)

class ImuHeading():
    def __init__(self):
        self.RH = ROSHandler(map)
        self.madgwick = Madgwick()
        self.initial_offset = 0
        self.curve_list = ['1', '7', '8', '9', '10', '11', '15', '16', '17', '21', '22', '23',
                           '24', '25', '26', '27', '36', '37', '38', '39', '43', '44', '54', '59',
                           '60', '61', '62', '63', '68', '69', '70', '72', '73', '78', '79', '80']

        self.imu_heading = None

    def euler_to_quaternion(self, roll, pitch, heading):
        cr = np.cos(roll / 2)
        sr = np.sin(roll / 2)
        cp = np.cos(pitch / 2)
        sp = np.sin(pitch / 2)
        cy = np.cos(heading / 2)
        sy = np.sin(heading / 2)

        q0 = cr * cp * cy + sr * sp * sy
        q1 = sr * cp * cy - cr * sp * sy
        q2 = cr * sp * cy + sr * cp * sy
        q3 = cr * cp * sy - sr * sp * cy

        return np.array([q0, q1, q2, q3])
    
    def check_msg_valid(self):
        nav_att_checklist = [self.RH.navatt.roll, self.RH.navatt.pitch, self.RH.navatt.heading] # excluded self.nav_velocity 
        imu_meas_checklist = [self.RH.imumeas.header, self.RH.imumeas.linear_acceleration, self.RH.imumeas.angular_velocity] # excluded self.nav_velocity_last

        nav_att_check = None not in nav_att_checklist
        imu_meas_check = None not in imu_meas_checklist

        checklist = [nav_att_check, imu_meas_check]
        result = False not in checklist

        return result

    def run(self):
        rate = rospy.Rate(30)
        
        last_s = None
        last_ns = None

        while not rospy.is_shutdown():
            if self.check_msg_valid():
                q = self.euler_to_quaternion(np.deg2rad(self.RH.navatt.roll), np.deg2rad(self.RH.navatt.pitch), np.deg2rad(self.RH.navatt.heading))
                last_s = self.RH.imumeas.header.stamp.secs
                last_ns = self.RH.imumeas.header.stamp.nsecs

            # heading from imu_meas
            accel = np.array([self.RH.imumeas.linear_acceleration.x, self.RH.imumeas.linear_acceleration.y, self.RH.imumeas.linear_acceleration.z])
            gyro = np.array([self.RH.imumeas.angular_velocity.x, self.RH.imumeas.angular_velocity.y, self.RH.imumeas.angular_velocity.z])
            
            # calculate dt
            ds = self.RH.imumeas.header.stamp.secs - last_s
            dns = self.RH.imumeas.header.stamp.nsecs - last_ns
            dt = ds + dns*1e-9
            last_s = self.RH.imumeas.header.stamp.secs
            last_ns = self.RH.imumeas.header.stamp.nsecs
            # print("dt:", dt)

            if self.RH.heading_fixed: # wrong heading
                q = self.madgwick.updateIMU(q=q, gyr=gyro, acc=accel, dt=dt)
                heading = -np.rad2deg(np.arctan2(2.0*(q[0]*q[3] + q[1]*q[2]), 1.0 - 2.0*(q[2]**2 + q[3]**2)))
            else: # right heading
                q = self.euler_to_quaternion(np.deg2rad(self.RH.navatt.roll), np.deg2rad(self.RH.navatt.pitch), np.deg2rad(self.RH.navatt.heading))
                self.cnt = 0
                heading = -np.rad2deg(np.arctan2(2.0*(q[0]*q[3] + q[1]*q[2]), 1.0 - 2.0*(q[2]**2 + q[3]**2)))
                self.initial_offset = self.RH.navatt.heading - heading
            # straight : -0.023
            # curve : -0.015
            if self.RH.curr_lane_id in self.curve_list:
                constant_offset = -0.015*self.cnt
                #print('CURVE IMU')
            else:
                constant_offset = -0.023*self.cnt
                #print('STRAIGHT IMU')
            
            heading += self.initial_offset
            offseted_heading = heading + constant_offset

            val = 0
            if offseted_heading < 0:
                val = 360
            if offseted_heading > 360:
                val = -360
            
            clipped_heading = -(offseted_heading + val - 90)%360
            
            self.cnt += 1

            self.RH.publish(clipped_heading)
            self.imu_heading = clipped_heading
            # print(f"                                compensated heading : {clipped_heading:.2f}")
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    imu_heading = ImuHeading()
    imu_heading.run()

if __name__ == "__main__":
    main()
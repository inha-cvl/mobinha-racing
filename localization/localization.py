import rospy

import sys
import signal
import numpy as np

from ros_handler import ROSHandler
from ahrs.filters import Madgwick

def signal_handler(sig, frame):
    sys.exit(0)

class Localization():
    def __init__(self):
        self.RH = ROSHandler(map)
        self.madgwick = Madgwick()
        self.condition = False

        self.curve_list = ['1', '7', '8', '9', '10', '11', '15', '16', '17', '21', '22', '23',
                           '24', '25', '26', '27', '36', '37', '38', '39', '43', '44', '54', '59',
                           '60', '61', '62', '63', '68', '69', '70', '72', '73', '78', '79', '80']

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

    def execute(self):
        rate = rospy.Rate(30)
        
        
        last_s = None
        last_ns = None
        initial_offset = 0
        initial_flag = True
        cnt = 0


        while not rospy.is_shutdown():
            while not self.condition:
                if self.RH.navpvt.condition and self.RH.navatt.condition and self.RH.imumeas.condition:
                    q = self.euler_to_quaternion(np.deg2rad(self.RH.navatt.roll), np.deg2rad(self.RH.navatt.pitch), np.deg2rad(self.RH.navatt.heading))
                    last_s = self.RH.imumeas.header.stamp.secs
                    last_ns = self.RH.imumeas.header.stamp.nsecs
                    self.condition = True
                
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

            if self.RH.heading_fixed:
                q = self.euler_to_quaternion(np.deg2rad(self.RH.navatt.roll), np.deg2rad(self.RH.navatt.pitch), np.deg2rad(self.RH.navatt.heading))
            else:
                q = self.madgwick.updateIMU(q=q, gyr=gyro, acc=accel, dt=dt)
            heading = -np.rad2deg(np.arctan2(2.0*(q[0]*q[3] + q[1]*q[2]), 1.0 - 2.0*(q[2]**2 + q[3]**2)))
            # straight : -0.023
            # curve : -0.015
            if self.RH.curr_lane_id in self.curve_list:
                constant_offset = -0.015*cnt
            else:
                constant_offset = -0.023*cnt
            
            heading += initial_offset
            offseted_heading = heading + constant_offset

            val = 0
            if offseted_heading < 0:
                val = 360
            if offseted_heading > 360:
                val = -360
            
            clipped_heading = offseted_heading + val
            cnt += 1

            if initial_flag:
                initial_offset = self.RH.navatt.heading - heading
                initial_flag = False

            self.RH.publish(clipped_heading)
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    localization = Localization()
    localization.execute()

if __name__ == "__main__":
    main()
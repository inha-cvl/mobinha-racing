import rospy
import sys
import signal
import numpy as np
import math
import matplotlib.pyplot as plt
from ahrs.filters import Madgwick
from filterpy.kalman import KalmanFilter
from pyproj import Proj, Transformer

from drive_msgs.msg import *
from std_msgs.msg import Float32
from ublox_msgs.msg import NavPVT, NavATT
from sensor_msgs.msg import Imu, NavSatFix
from navatt_subs import NAVATT
from navpvt_subs import NAVPVT
from imu_meas_subs import IMUMEAS

from datetime import datetime

def signal_handler(sig, frame):
    plt.pause(100)
    sys.exit(0)

class ROSHandler:
    def __init__(self):
        rospy.init_node('localization', anonymous=False)
        self.set_values()
        self.set_protocol()

    def set_values(self):
        self.heading_fixed = False
        self.navatt = NAVATT()
        self.navpvt = NAVPVT()
        self.imumeas = IMUMEAS()
        self.map_name = None
        self.transformer = None
        self.local_pose = [0, 0]
        self.curr_lane_id = None
        
        self.nav_header = None
        self.nav_pos = None
        self.nav_heading = None

        self.nav_header_last = None
        self.nav_pos_last = None
        self.nav_heading_last = None
        self.nav_velocity_last = None

        self.can_velocity = None
        self.can_steer = None
        self.corr_can_velocity = None

        self.can_velocity_last = None
        self.can_steer_last = None

        self.s_params = [-1.69519446e-01, 3.14832448e-02, -2.42469118e-04, 1.68413777e-06]
        self.c_params = [-3.04750083e-01, 4.43420297e-02, -6.07069742e-04, 4.46079605e-06]
        self.params = [-8.38357609e-03, 2.37367164e-02, -1.59672708e-04, 1.53623118e-06]
        self.steer_scale_factor = 36.2 / 500

        base_lla = [35.65492524, 128.39351431, 7]
        proj_wgs84 = Proj(proj='latlong', datum='WGS84')
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=base_lla[0], lon_0=base_lla[1], h_0=base_lla[2])
        self.transformer = Transformer.from_proj(proj_wgs84, proj_enu)

    def set_protocol(self):
        rospy.Subscriber("/ublox/navatt", NavATT, self.navatt.callback)
        rospy.Subscriber("/ublox/navpvt", NavPVT, self.navpvt_cb)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.fix_cb)
        rospy.Subscriber("/ublox/imu_meas", Imu, self.imumeas.callback)
        rospy.Subscriber('/CANOutput', CANOutput, self.canoutput_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        rospy.Subscriber('/LaneData', LaneData, self.lanedata_cb)
        
        self.fix_heading_pub = rospy.Publisher('/localization/heading', Float32, queue_size=1)
        self.kf_heading_pub = rospy.Publisher('/kf/heading', Float32, queue_size=1)
        self.kf_position_pub = rospy.Publisher('/kf/position', Float32, queue_size=1)

    def navpvt_cb(self, msg): 
        self.navpvt_update()
        lat = msg.lat * 1e-7
        lon = msg.lon * 1e-7
        x, y, _ = self.transformer.transform(lon, lat, 7)
        self.nav_pos = (x, y)
        self.nav_heading = -(msg.heading * 1e-5 - 90) % 360
    
    def navpvt_update(self):
        self.nav_pos_last = self.nav_pos
        self.nav_heading_last = self.nav_heading
    
    def fix_cb(self, msg):
        self.fix_update()
        self.nav_header = msg.header

    def fix_update(self):
        self.nav_header_last = self.nav_header

    def system_status_cb(self, msg):
        if msg.headingSet.data == 1:
            self.heading_fixed = True 
        else:
            self.heading_fixed = False
    
    def canoutput_cb(self, msg): 
        self.canoutput_update()
        vRR = float(msg.WHEEL_SPD_RR.data)
        vRL = float(msg.WHEEL_SPD_RL.data) 
        self.can_velocity = (vRR + vRL) / 7.2
        self.corr_can_velocity = (self.can_velocity * 3.6 +
                                  self.params[0] + self.params[1] * (self.can_velocity * 3.6) +
                                  self.params[2] * ((self.can_velocity * 3.6) ** 2) +
                                  self.params[3] * ((self.can_velocity * 3.6) ** 3)) / 3.6

        handle_ang = float(msg.StrAng.data)
        self.can_steer = handle_ang * self.steer_scale_factor 
    
    def canoutput_update(self):
        self.can_velocity_last = self.can_velocity
        self.corr_can_velocity_last = self.corr_can_velocity
        self.can_steer_last = self.can_steer

    def lanedata_cb(self, msg):
        self.curr_lane_id = str(msg.currentLane.id.data)

    def publish(self, heading, position):
        if heading is not None:
            self.kf_heading_pub.publish(Float32(heading))
        if position is not None:
            self.kf_position_pub.publish(Float32(position))

class ImuHeading:
    def __init__(self):
        self.madgwick = Madgwick()
        self.RH = ROSHandler()
        self.condition = False
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

    def execute(self):
        rate = rospy.Rate(30)
        
        last_s = None
        last_ns = None
        self.initial_offset = 0
        self.initial_flag = True

        while not rospy.is_shutdown():
            while not self.condition:
                if self.RH.navpvt.condition and self.RH.navatt.condition and self.RH.imumeas.condition:
                    q = self.euler_to_quaternion(np.deg2rad(self.RH.navatt.roll), np.deg2rad(self.RH.navatt.pitch), np.deg2rad(self.RH.navatt.heading))
                    last_s = self.RH.imumeas.header.stamp.secs
                    last_ns = self.RH.imumeas.header.stamp.nsecs
                    self.condition = True
                
            accel = np.array([self.RH.imumeas.linear_acceleration.x, self.RH.imumeas.linear_acceleration.y, self.RH.imumeas.linear_acceleration.z])
            gyro = np.array([self.RH.imumeas.angular_velocity.x, self.RH.imumeas.angular_velocity.y, self.RH.imumeas.angular_velocity.z])
            
            ds = self.RH.imumeas.header.stamp.secs - last_s
            dns = self.RH.imumeas.header.stamp.nsecs - last_ns
            dt = ds + dns * 1e-9
            last_s = self.RH.imumeas.header.stamp.secs
            last_ns = self.RH.imumeas.header.stamp.nsecs

            if self.RH.heading_fixed:
                q = self.madgwick.updateIMU(q=q, gyr=gyro, acc=accel, dt=dt)
                heading = -np.rad2deg(np.arctan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2)))
            else:
                q = self.euler_to_quaternion(np.deg2rad(self.RH.navatt.roll), np.deg2rad(self.RH.navatt.pitch), np.deg2rad(self.RH.navatt.heading))
                self.cnt = 0
                heading = -np.rad2deg(np.arctan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2)))
                self.initial_offset = self.RH.navatt.heading - heading

            if self.RH.curr_lane_id in self.curve_list:
                constant_offset = -0.015 * self.cnt
            else:
                constant_offset = -0.023 * self.cnt
            
            heading += self.initial_offset
            offseted_heading = heading + constant_offset

            val = 0
            if offseted_heading < 0:
                val = 360
            if offseted_heading > 360:
                val = -360
            
            clipped_heading = offseted_heading + val
            self.cnt += 1

            self.RH.publish(clipped_heading, None)
            self.imu_heading = clipped_heading
            rate.sleep()

class DR_BICYCLE:
    def __init__(self):
        self.RH = ROSHandler()
        self.wheelbase = 2.72
        self.steer_scale_factor = 36.2 / 500

        self.s_params = [-1.69519446e-01, 3.14832448e-02, -2.42469118e-04, 1.68413777e-06]
        self.c_params = [-3.04750083e-01, 4.43420297e-02, -6.07069742e-04, 4.46079605e-06]
        self.params = [-8.38357609e-03, 2.37367164e-02, -1.59672708e-04, 1.53623118e-06]

        base_lla = [35.65492524, 128.39351431, 7]
        proj_wgs84 = Proj(proj='latlong', datum='WGS84')
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=base_lla[0], lon_0=base_lla[1], h_0=base_lla[2])
        self.transformer = Transformer.from_proj(proj_wgs84, proj_enu)

        self.dt = None
        self.dr_pos = None
        self.dr_heading = None   

    def check_msg_valid(self):
        nav_checklist = [self.RH.nav_header, self.RH.nav_heading, self.RH.nav_pos]
        nav_last_checklist = [self.RH.nav_header_last, self.RH.nav_heading_last, self.RH.nav_pos_last]
        can_checklist = [self.RH.can_steer, self.RH.can_velocity]
        can_last_checklist = [self.RH.can_steer_last, self.RH.can_velocity_last]

        nav_check = None not in nav_checklist
        nav_last_check = None not in nav_last_checklist
        can_check = None not in can_checklist
        can_last_check = None not in can_last_checklist

        checklist = [nav_check, nav_last_check, can_check, can_last_check]
        result = False not in checklist

        return result
    
    def calculate_dt(self):
        now = self.RH.nav_header.stamp
        last = self.RH.nav_header_last.stamp
        self.dt = now.to_sec() - last.to_sec()

    def calculate_nav_v(self):
        self.RH.nav_velocity_last = math.sqrt((self.RH.nav_pos[0] - self.RH.nav_pos_last[0]) ** 2 + (self.RH.nav_pos[1] - self.RH.nav_pos_last[1]) ** 2) / self.dt    

    def calculate_nextpos(self):
        theta_rad = math.radians(self.RH.nav_heading_last)
        delta_rad = math.radians(self.RH.can_steer_last)

        start_pos = self.RH.nav_pos_last
        dr_x = start_pos[0] + self.dt * self.RH.corr_can_velocity_last * math.cos(theta_rad)
        dr_y = start_pos[1] + self.dt * self.RH.corr_can_velocity_last * math.sin(theta_rad)
        self.dr_pos = (dr_x, dr_y)
        self.dr_heading = self.RH.nav_heading_last + self.dt * math.degrees((self.RH.corr_can_velocity_last / self.wheelbase) * math.tan(delta_rad))

    def is_available(self):
        result = None not in [self.dr_pos, self.dr_heading]
        return result
    
    def run(self): 
        if self.check_msg_valid():
            self.calculate_dt()
            self.calculate_nextpos() 
            self.calculate_nav_v()

class KFLocalization:
    def __init__(self):
        self.RH = ROSHandler()
        self.DR = DR_BICYCLE()
        self.IH = ImuHeading()

        self.nav_pos = None
        self.nav_heading = None
        self.dr_pos = None
        self.dr_heading = None
        self.imu_heading = None
        
        self.lidar_pos = None  # TODO
        self.lidar_heading = None  # TODO
        
        self.kf_heading = self.init_heading_kalman_filter()
        self.kf_position = self.init_position_kalman_filter()

        self.nav_positions = []
        self.filtered_positions = []
        self.nav_headings = []
        self.filtered_headings = []
        self.position_diffs = []
        self.heading_diffs = []

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
        self.nav_heading = self.RH.nav_heading
        self.dr_pos = self.DR.dr_pos
        self.dr_heading = self.DR.dr_heading
        self.imu_heading = self.IH.imu_heading
        
        # TODO: Update self.lidar_pos and self.lidar_heading with Lidar data

    def run(self):
        plt.ion()
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12))
        
        while not rospy.is_shutdown():
            self.update_sensors()

            filtered_heading = None
            filtered_position = None

            if self.nav_heading is not None:
                self.kf_heading.predict()
                self.kf_heading.update(self.nav_heading)
                filtered_heading = self.kf_heading.x[0]
                self.nav_headings.append(self.nav_heading)
                self.filtered_headings.append(filtered_heading)
                self.heading_diffs.append(abs(self.nav_heading - filtered_heading))
                rospy.loginfo(f"Filtered Heading: {filtered_heading}")

            if self.nav_pos is not None:
                self.kf_position.predict()
                self.kf_position.update(np.array([self.nav_pos[0], self.nav_pos[1]]))
                filtered_position = self.kf_position.x[:2]
                self.nav_positions.append(self.nav_pos)
                self.filtered_positions.append(filtered_position)
                self.position_diffs.append(np.linalg.norm(np.array(self.nav_pos) - np.array(filtered_position)))
                rospy.loginfo(f"Filtered Position: {filtered_position}")

            self.RH.publish(filtered_heading, filtered_position)

            # Plotting
            if len(self.nav_positions) > 30:
                self.nav_positions.pop(0)
                self.filtered_positions.pop(0)
                self.position_diffs.pop(0)
                self.nav_headings.pop(0)
                self.filtered_headings.pop(0)
                self.heading_diffs.pop(0)

            ax1.clear()
            ax1.plot([pos[0] for pos in self.nav_positions], [pos[1] for pos in self.nav_positions], 'bo-', label='Nav Position')
            ax1.plot([pos[0] for pos in self.filtered_positions], [pos[1] for pos in self.filtered_positions], 'ro-', label='Filtered Position')
            ax1.legend()
            ax1.set_title('Real-time Navigation vs Filtered Position')
            ax1.set_xlabel('X (m)')
            ax1.set_ylabel('Y (m)')
            ax1.grid(True)

            ax2.clear()
            ax2.plot(self.position_diffs, 'g-', label='Position Difference')
            ax2.legend()
            ax2.set_title('Nav Position vs Filtered Position Difference')
            ax2.set_xlabel('Time Steps')
            ax2.set_ylabel('Position Difference (m)')
            ax2.grid(True)

            ax3.clear()
            ax3.plot(self.heading_diffs, 'y-', label='Heading Difference')
            ax3.legend()
            ax3.set_title('Nav Heading vs Filtered Heading Difference')
            ax3.set_xlabel('Time Steps')
            ax3.set_ylabel('Heading Difference (degrees)')
            ax3.grid(True)

            plt.draw()
            plt.pause(0.1)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    kf_localization = KFLocalization()
    kf_localization.run()

if __name__ == "__main__":
    main()

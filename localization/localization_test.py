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
from geometry_msgs.msg import Pose2D
from ublox_msgs.msg import NavPVT, NavATT
from sensor_msgs.msg import Imu, NavSatFix

from datetime import datetime

def signal_handler(sig, frame):
    sys.exit(0)

class ROSHandler():
    def __init__(self):
        rospy.init_node('localization', anonymous=False)
        self.set_values()
        self.set_protocol()
        self.set_params()

    def set_values(self):
        self.heading_fixed = False
        self.transformer = None
        self.local_pose = [0,0]   
        self.curr_lane_id = None
        
        self.nav_header = None
        self.nav_pos = [None, None]
        self.nav_heading = None
        self.nav_roll = None
        self.nav_pitch = None

        self.nav_header_last = None
        self.nav_pos_last = [None, None]
        self.nav_heading_last = None

        self.imu_header = None
        self.imu_angular_velocity = None
        self.imu_linear_acceleration = None

        self.can_velocity = None
        self.can_steer = None
        self.corr_can_velocity = None

        self.can_velocity_last = None
        self.can_steer_last = None
        self.corr_can_velocity_last = None

    def set_protocol(self):
        rospy.Subscriber('/ublox/navatt', NavATT, self.navatt_cb)
        rospy.Subscriber('/ublox/navpvt', NavPVT, self.navpvt_cb)
        rospy.Subscriber('/ublox/imu_meas', Imu, self.imu_cb)
        rospy.Subscriber('/ublox/fix', NavSatFix, self.fix_cb)
        rospy.Subscriber('/CANOutput', CANOutput, self.canoutput_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        rospy.Subscriber('/LaneData', LaneData, self.lanedata_cb)
        
        self.kf_pose_pub = rospy.Publisher('/kf/pose', Pose2D, queue_size=1)
    
    def set_params(self):
        self.steer_scale_factor = 36.2/500

        self.s_params = [-1.69519446e-01, 3.14832448e-02, -2.42469118e-04, 1.68413777e-06]
        self.c_params = [-3.04750083e-01, 4.43420297e-02, -6.07069742e-04, 4.46079605e-06]
        self.params = [-8.38357609e-03, 2.37367164e-02, -1.59672708e-04, 1.53623118e-06]

        base_lla = [35.65492524, 128.39351431, 7] # KIAPI_Racing base
        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=base_lla[0], lon_0=base_lla[1], h_0=base_lla[2])
        self.transformer = Transformer.from_proj(proj_wgs84, proj_enu)

    def navatt_cb(self, msg):  # gain heading
        self.nav_heading_last = self.nav_heading
        self.nav_heading = -(msg.heading*1e-5 - 90)%360 
        self.nav_roll = msg.roll*1e-5
        self.nav_pitch = msg.pitch*1e-5

    def navpvt_cb(self, msg): # gain position
        self.nav_pos_last = self.nav_pos
        lat = msg.lat*1e-7
        lon = msg.lon*1e-7
        x, y, _= self.transformer.transform(lon, lat, 7)
        self.nav_pos = [x, y]
    
    def imu_cb(self, msg):
        self.imu_header = msg.header
        self.imu_angular_velocity = msg.angular_velocity
        self.imu_linear_acceleration = msg.linear_acceleration
    
    def fix_cb(self, msg): # gain header
        self.nav_header_last = self.nav_header
        self.nav_header = msg.header

    def system_status_cb(self, msg):
        if msg.headingSet.data == 1:
            self.heading_fixed = True 
        else:
            self.heading_fixed = False
    
    def canoutput_cb(self, msg): # gain velocity, steering angle
        self.canoutput_update()
        vRR = float(msg.WHEEL_SPD_RR.data)
        vRL = float(msg.WHEEL_SPD_RL.data) 
        self.can_velocity = (vRR + vRL)/7.2 # [m/s]
        self.corr_can_velocity = (self.can_velocity*3.6 \
                                  + self.params[0] + self.params[1]*(self.can_velocity*3.6) \
                                    + self.params[2]*((self.can_velocity*3.6)**2) \
                                        + self.params[3]*((self.can_velocity*3.6)**3))/3.6 # [m/s]

        handle_ang = float(msg.StrAng.data)
        self.can_steer = handle_ang*self.steer_scale_factor 
    
    def canoutput_update(self):
        self.can_velocity_last = self.can_velocity
        self.corr_can_velocity_last = self.corr_can_velocity
        self.can_steer_last = self.can_steer

    def lanedata_cb(self, msg):
        self.curr_lane_id = str(msg.currentLane.id.data)

    def publish(self, heading, position):
        pos_msg = Pose2D()
        pos_msg.x = position[0]
        pos_msg.y = position[1]
        pos_msg.theta = heading
        self.kf_pose_pub.publish(pos_msg)

class ImuHeading():
    def __init__(self):
        self.RH = ROSHandler()
        self.madgwick = Madgwick()
        self.initial_offset = 0
        self.curve_list = ['1', '7', '8', '9', '10', '11', '15', '16', '17', '21', '22', '23',
                           '24', '25', '26', '27', '36', '37', '38', '39', '43', '44', '54', '59',
                           '60', '61', '62', '63', '68', '69', '70', '72', '73', '78', '79', '80']

        self.imu_corr_heading = None

        self.last_s = None
        self.last_ns = None

        self.initial = True

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
        nav_att_checklist = [self.RH.nav_roll, self.RH.nav_pitch, self.RH.nav_heading]
        imu_checklist = [self.RH.imu_header, self.RH.imu_linear_acceleration, self.RH.imu_angular_velocity]

        nav_att_check = None not in nav_att_checklist
        imu_check = None not in imu_checklist

        checklist = [nav_att_check, imu_check]
        result = False not in checklist

        return result

    def run(self):

        if self.check_msg_valid():
            if self.initial:
                q = self.euler_to_quaternion(np.deg2rad(self.RH.nav_roll), np.deg2rad(self.RH.nav_pitch), np.deg2rad(self.RH.nav_heading))
                self.last_s = self.RH.imu_header.stamp.secs
                self.last_ns = self.RH.imu_header.stamp.nsecs
                self.initial = False
            

            # heading from imu
            accel = np.array([self.RH.imu_linear_acceleration.x, self.RH.imu_linear_acceleration.y, self.RH.imu_linear_acceleration.z])
            gyro = np.array([self.RH.imu_angular_velocity.x, self.RH.imu_angular_velocity.y, self.RH.imu_angular_velocity.z])
            
            # calculate dt
            ds = self.RH.imu_header.stamp.secs - self.last_s
            dns = self.RH.imu_header.stamp.nsecs - self.last_ns
            dt = ds + dns*1e-9
            self.last_s = self.RH.imu_header.stamp.secs
            self.last_ns = self.RH.imu_header.stamp.nsecs

            if self.RH.heading_fixed: # wrong heading
                q = self.madgwick.updateIMU(q=q, gyr=gyro, acc=accel, dt=dt)
                heading = -np.rad2deg(np.arctan2(2.0*(q[0]*q[3] + q[1]*q[2]), 1.0 - 2.0*(q[2]**2 + q[3]**2)))
            else: # right heading
                q = self.euler_to_quaternion(np.deg2rad(self.RH.nav_roll), np.deg2rad(self.RH.nav_pitch), np.deg2rad(self.RH.nav_heading))
                self.cnt = 0
                heading = -np.rad2deg(np.arctan2(2.0*(q[0]*q[3] + q[1]*q[2]), 1.0 - 2.0*(q[2]**2 + q[3]**2)))
                self.initial_offset = self.RH.nav_heading - heading
            # straight : -0.023
            # curve : -0.015
            if self.RH.curr_lane_id in self.curve_list:
                constant_offset = -0.015*self.cnt
            else:
                constant_offset = -0.023*self.cnt

            heading += self.initial_offset
            offseted_heading = heading + constant_offset

            val = 0
            if offseted_heading < 0:
                val = 360
            if offseted_heading > 360:
                val = -360
            
            clipped_heading = -(offseted_heading + val - 90)%360
            
            self.cnt += 1

            self.imu_corr_heading = clipped_heading

class DR_BICYCLE:
    def __init__(self):
        self.RH = ROSHandler()

        self.wheelbase = 2.72

        # LLH to ENU
        base_lla = [35.65492524, 128.39351431, 7] # KIAPI_Racing base
        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=base_lla[0], lon_0=base_lla[1], h_0=base_lla[2])
        self.transformer = Transformer.from_proj(proj_wgs84, proj_enu)

        # dt
        self.dt = None # [sec]

        # result
        self.dr_pos = None
        self.dr_heading = None   

    def check_msg_valid(self):
        nav_checklist = [self.RH.nav_header, self.RH.nav_heading, self.RH.nav_pos] # excluded self.nav_velocity 
        nav_last_checklist = [self.RH.nav_header_last, self.RH.nav_heading_last, self.RH.nav_pos_last] # excluded self.nav_velocity_last
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
        self.dt = now.to_sec() - last.to_sec() # [sec] 

    def calculate_nextpos(self):
        theta_rad = math.radians(self.RH.nav_heading_last)
        delta_rad = math.radians(self.RH.can_steer_last)

        start_pos = self.RH.nav_pos_last
        dr_x = start_pos[0] + self.dt * self.RH.corr_can_velocity_last * math.cos(theta_rad)
        dr_y = start_pos[1] + self.dt * self.RH.corr_can_velocity_last * math.sin(theta_rad)
        self.dr_pos = [dr_x, dr_y]
        self.dr_heading = self.RH.nav_heading_last + self.dt * math.degrees((self.RH.corr_can_velocity_last / self.wheelbase) * math.tan(delta_rad))

    def is_available(self):
        result = None not in [self.dr_pos, self.dr_heading]
        return result
    
    def run(self): 
        # rate = rospy.Rate(20)
        # while not rospy.is_shutdown():
        if self.check_msg_valid():
            self.calculate_dt()
            self.calculate_nextpos() 
            # rate.sleep()

class KFLocalization:
    def __init__(self):
        self.initailize = True
        self.RH = ROSHandler()
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

        # matplotlib
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1)
        self.nav_positions = []
        self.filtered_positions = []
        self.position_diffs = []
        self.heading_diffs = []

        self.filtered_pub = rospy.Publisher('/test_position', Pose2D, queue_size=1)
        
    def update_sensors(self):
        self.nav_heading_last = self.nav_heading
        self.nav_pos_last = self.nav_pos
        self.dr_heading_last = self.dr_heading
        self.dr_pos_last = self.dr_pos
        self.imu_heading_last = self.imu_heading

        self.nav_pos = self.RH.nav_pos
        self.nav_heading = self.RH.nav_heading

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
        self.p_nav_heading = self.nav_heading - self.nav_cw_cnt * 360

        if self.imu_heading - self.imu_heading_last < -355:
            self.imu_cw_cnt -= 1
        if self.imu_heading - self.imu_heading_last > 355:
            self.imu_cw_cnt += 1
        self.p_imu_heading = self.imu_heading - self.imu_cw_cnt * 360

        if self.dr_heading - self.dr_heading_last < -355:
            self.dr_cw_cnt -= 1
        if self.dr_heading - self.dr_heading_last > 355:
            self.dr_cw_cnt += 1
        self.p_dr_heading = self.dr_heading - self.dr_cw_cnt * 360

    def init_heading_kalman_filter(self):
        kf = KalmanFilter(dim_x=1, dim_z=3)  # dim: state vector=1, measurement vector=3
        kf.x = np.ones(1) * 1000  # initial heading value
        kf.P = np.eye(1) * 1000.  # initial cov matrix
        kf.R = np.diag([1, 1, 1])  # initial meas_cov matrix
        kf.Q = np.eye(1) * 0.01  # inital process noise matrix
        kf.F = np.eye(1)  # state transition matrix
        kf.H = np.array([[1], [1], [1]])
        return kf

    def init_pos_kalman_filter(self):
        kf = KalmanFilter(dim_x=2, dim_z=4)
        kf.x = np.ones(2) * 1000
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
            

            self.filtered_positions.append(self.filtered_pos if self.filtered_pos is not None else 0)
            self.nav_positions.append(self.nav_pos if self.nav_pos is not None else 0)
            
            filtered_pos_msg = Pose2D()
            filtered_pos_msg.x = self.filtered_pos[0] if self.filtered_pos is not None else 0
            filtered_pos_msg.y = self.filtered_pos[1] if self.filtered_pos is not None else 0
            filtered_pos_msg.theta = self.filtered_heading if self.filtered_heading is not None else 0
            self.filtered_pub.publish(filtered_pos_msg)
            
            print("Nav heading:", self.nav_heading)
            print("IMU heading:", self.imu_heading)
            print("DR heading:", self.dr_heading)
            print("Filtered heading:", self.filtered_heading)
            print("Nav position:", self.nav_pos)
            print("DR position:", self.dr_pos)
            print("Filtered position:", self.filtered_pos)
            print("----------------------")

            self.ax1.clear()
            self.ax1.plot([pos[0] for pos in self.nav_positions], [pos[1] for pos in self.nav_positions], 'bo-', label='Nav Position')
            self.ax1.plot([pos[0] for pos in self.filtered_positions], [pos[1] for pos in self.filtered_positions], 'ro-', label='Filtered Position')
            self.ax1.legend()
            self.ax1.set_title('Real-time Navigation vs Filtered Position')
            self.ax1.set_xlabel('X (m)')
            self.ax1.set_ylabel('Y (m)')
            self.ax1.grid(True)

            self.ax2.clear()
            self.ax2.plot(self.position_diffs, 'g-', label='Position Difference')
            self.ax2.legend()
            self.ax2.set_title('Nav Position vs Filtered Position Difference')
            self.ax2.set_xlabel('Time Steps')
            self.ax2.set_ylabel('Position Difference (m)')
            self.ax2.grid(True)

            self.ax3.clear()
            self.ax3.plot(self.heading_diffs, 'y-', label='Heading Difference')
            self.ax3.legend()
            self.ax3.set_title('Nav Heading vs Filtered Heading Difference')
            self.ax3.set_xlabel('Time Steps')
            self.ax3.set_ylabel('Heading Difference (degrees)')
            self.ax3.grid(True)

            plt.draw()
            plt.pause(0.1)

            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    kf_localization = KFLocalization()
    kf_localization.run()

if __name__ == "__main__":
    main()

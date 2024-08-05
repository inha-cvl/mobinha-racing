import rospy
import sys
import signal
import numpy as np
import math
import matplotlib.pyplot as plt
from ahrs.filters import Madgwick
# from filterpy.kalman import KalmanFilter
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
        
        self.best_pose_pub = rospy.Publisher('/best/pose', Pose2D, queue_size=1)
    
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
        self.best_pose_pub.publish(pos_msg)

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

        self.q = None

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

    def run(self, best_heading, nav_valid):

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

            if not nav_valid: # wrong heading
                self.q = self.madgwick.updateIMU(q=self.q, gyr=gyro, acc=accel, dt=dt)
                heading = -np.rad2deg(np.arctan2(2.0*(self.q[0]*self.q[3] + self.q[1]*self.q[2]), 1.0 - 2.0*(self.q[2]**2 + self.q[3]**2)))
            else: # right heading
                self.q = self.euler_to_quaternion(np.deg2rad(self.RH.nav_roll), np.deg2rad(self.RH.nav_pitch), np.deg2rad(best_heading))
                self.cnt = 0
                heading = -np.rad2deg(np.arctan2(2.0*(self.q[0]*self.q[3] + self.q[1]*self.q[2]), 1.0 - 2.0*(self.q[2]**2 + self.q[3]**2)))
                self.initial_offset = best_heading - heading
            # straight : -0.023
            # curve : -0.015
            if self.RH.curr_lane_id in self.curve_list:
                constant_offset = -0.015*self.cnt
            else:
                constant_offset = -0.023*self.cnt

            heading += self.initial_offset
            offseted_heading = heading + constant_offset

            # val = 0
            # if offseted_heading < 0:
            #     val = 360
            # if offseted_heading > 360:
            #     val = -360
            
            # clipped_heading = -(offseted_heading + val - 90)%360
            
            self.cnt += 1

            self.imu_corr_heading = offseted_heading

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
        self.dr_pos = [None, None]
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

    def calculate_nextpos(self, best_hdg_last, best_pos_last, nav_valid):
        if nav_valid:
            delta_rad = math.radians(self.RH.can_steer_last)

            theta_rad = math.radians(self.RH.nav_heading_last) if best_hdg_last is None else best_hdg_last
            start_pos = self.RH.nav_pos_last if best_pos_last[0] is None else best_pos_last

            dr_x = start_pos[0] + self.dt * self.RH.corr_can_velocity_last * math.cos(theta_rad)
            dr_y = start_pos[1] + self.dt * self.RH.corr_can_velocity_last * math.sin(theta_rad)
            self.dr_pos = [dr_x, dr_y]
            self.dr_heading = self.RH.nav_heading_last + self.dt * math.degrees((self.RH.corr_can_velocity_last / self.wheelbase) * math.tan(delta_rad))


    # def is_available(self):
    #     result = None not in [self.dr_pos[0], self.dr_heading]
    #     return result
    
    def run(self, best_heading_last, best_pos_last, nav_valid): 
        if self.check_msg_valid():
            self.calculate_dt()
            self.calculate_nextpos(best_heading_last, best_pos_last, nav_valid) 

class BestLocalization:
    def __init__(self):
        self.initialize = True
        self.RH = ROSHandler()
        self.DR = DR_BICYCLE()
        self.IH = ImuHeading()

        self.nav_heading = None
        self.nav_pos = None
        self.dr_heading = None
        self.dr_pos = [None, None]
        self.imu_heading = None

        self.best_heading = None
        self.best_pos = [None, None]
        
        self.nav_heading_last = None
        self.nav_pos_last = [None, None]
        self.dr_heading_last = None
        self.dr_pos_last = [None, None]
        self.imu_heading_last = None

        self.best_heading_last = None
        self.best_pos_last = [None, None]

        self.nav_cw_cnt = 0
        self.imu_cw_cnt = 0
        self.dr_cw_cnt = 0
        self.p_nav_heading = None
        self.p_imu_heading = None
        self.p_dr_heading = None

        self.nav_valid = True

        # matplotlib
        plt.ion()
        # self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1)
        self.fig, self.ax1 = plt.subplots()
        self.nav_positions = []
        self.best_positions = []
        self.position_diffs = []
        self.heading_diffs = []

        self.best_pub = rospy.Publisher('/test_position', Pose2D, queue_size=1)

        # self.update_sensors()
        
    def update_sensors(self):
        self.nav_heading_last = self.nav_heading
        self.nav_pos_last = self.nav_pos
        self.dr_heading_last = self.dr_heading
        self.dr_pos_last = self.dr_pos
        self.imu_heading_last = self.imu_heading
        self.best_heading_last = self.best_heading
        self.best_pos_last = self.best_pos

        self.nav_heading = self.RH.nav_heading
        self.nav_pos = self.RH.nav_pos
        self.dr_heading = self.DR.dr_heading
        self.dr_pos = self.DR.dr_pos
        self.imu_heading = self.IH.imu_corr_heading
    
    def valid_hdg(self, hdg_last, hdg_now, hz): # not used yet, variable 'diff' needs field test
        if None in [hdg_last, hdg_now]:
            return False
        
        val = abs(hdg_last - hdg_now)
        diff = min(val, 360 - val)
        result = diff < 5
        
        return result

    def valid_pos(self, pos_last, pos_now, hz): # not used yet, variable 'diff' needs field test
        if None in [pos_last[0], pos_now[0]]:
            return False
        
        diff = ((pos_now[0]-pos_last[0])**2 + (pos_now[1]-pos_last[1])**2)**0.5
        result = diff < 5
        
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

    def integrate_heading(self, hz):
        self.nav_valid = self.valid_hdg(self.best_heading_last, self.nav_heading, hz)
        imu_valid = self.valid_hdg(self.best_heading_last, self.imu_heading, hz)
        dr_valid = self.valid_hdg(self.best_heading_last, self.dr_heading, hz)

        imu_weight = 0.5
        dr_weight = 0.5
        if not imu_valid:
            imu_weight = 0
        if not dr_valid:
            dr_weight = 0
        
        if self.nav_valid:
            self.best_heading = self.p_nav_heading
        else:
            print("nav not valid !!!!")
            self.best_heading = (self.p_imu_heading*imu_weight + self.p_dr_heading*dr_weight)/(imu_weight+dr_weight)

    def integrate_position(self, hz):
        self.nav_valid = self.valid_pos(self.nav_pos_last, self.nav_pos, hz)
        dr_valid = self.valid_pos(self.dr_pos_last, self.dr_pos, hz)

        if self.nav_valid:
            self.best_pos = self.nav_pos
        elif not self.nav_valid and dr_valid:
            self.best_pos = self.dr_pos
        else:
            pass
    
    def initiate(self):
        while self.nav_heading_last is None:
            self.best_heading = self.nav_heading
            self.best_pos = self.nav_pos
            self.update_sensors()

    def run(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.initiate()

            self.DR.run(self.best_heading_last, self.best_pos_last, self.nav_valid)
            self.IH.run(self.best_heading, self.nav_valid)

            self.update_sensors()
            if None not in [self.nav_heading_last, self.imu_heading_last, self.dr_heading_last]:
                self.heading_postprocess()
                self.integrate_heading(20)      

            if self.nav_pos_last[0] is not None and self.dr_pos_last[0] is not None:
                self.integrate_position(20)

            if None not in [self.best_heading, self.best_pos[0]]:
                self.RH.publish(self.best_heading, self.best_pos)
            
            rate.sleep()
            
            print("Nav heading:", self.p_nav_heading)
            print("IMU heading:", self.p_imu_heading)
            print("DR heading:", self.p_dr_heading)
            print("Best heading:", self.best_heading)

            print("Nav position:", self.nav_pos)
            print("DR position:", self.dr_pos)
            print("Best position:", self.best_pos)
            print("----------------------")

            self.ax1.clear()
            self.ax1.plot([pos[0] for pos in self.nav_positions], [pos[1] for pos in self.nav_positions], 'bo-', label='Nav Position')
            self.ax1.plot([pos[0] for pos in self.best_positions], [pos[1] for pos in self.best_positions], 'ro-', label='Best Position')
            self.ax1.legend()
            self.ax1.set_title('Real-time Navigation vs Best Position')
            self.ax1.set_xlabel('X (m)')
            self.ax1.set_ylabel('Y (m)')
            self.ax1.grid(True)

            # self.ax2.clear()
            # self.ax2.plot(self.position_diffs, 'g-', label='Position Difference')
            # self.ax2.legend()
            # self.ax2.set_title('Nav Position vs Best Position Difference')
            # self.ax2.set_xlabel('Time Steps')
            # self.ax2.set_ylabel('Position Difference (m)')
            # self.ax2.grid(True)

            # self.ax3.clear()
            # self.ax3.plot(self.heading_diffs, 'y-', label='Heading Difference')
            # self.ax3.legend()
            # self.ax3.set_title('Nav Heading vs Best Heading Difference')
            # self.ax3.set_xlabel('Time Steps')
            # self.ax3.set_ylabel('Heading Difference (degrees)')
            # self.ax3.grid(True)

            plt.draw()
            plt.pause(0.1)

            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    best_localization = BestLocalization()
    best_localization.run()

if __name__ == "__main__":
    main()


import math
import rospy
from ublox_msgs.msg import NavATT, NavPVT
from drive_msgs.msg import VehicleState, CANOutput, LaneData
from sensor_msgs.msg import Imu
from pyproj import Proj, Transformer
from ahrs.filters import Madgwick
import matplotlib.pyplot as plt
import numpy as np

import time
import sys
import signal

from ros_handler import ROSHandler

nav_poss = []
dr_poss = []
nav_hdgs = []
dr_hdgs = []
imu_hdgs = []

class Localization:
    def __init__(self):
        self.RH = ROSHandler()
        self.initiated = False

        self.last_pos = None
        self.last_hdg = None

        self.dr_pos = None
        self.dr_hdg = None
        # self.imu_hdg = None

        self.nav_hdg_invalid_cnt = 0
        self.nav_pos_invalid_cnt = 0

        self.hdg_mct = 5  # min compensation time
        self.pos_mct = 5

        # Initialize plot
        self.fig, self.ax = plt.subplots()
        self.nav_plot, = self.ax.plot([], [], 'bo-', label='NAV')
        self.dr_plot, = self.ax.plot([], [], 'ro-', label='DR')
        self.ax.legend()
        # self.ax.set_xlim(-100, 100)
        # self.ax.set_ylim(-100, 100)
        self.ax.axis("equal")

    def imu_timer(self, sec):
        if time.time() - self.imu_time > sec:
            self.imu_time = time.time()
            # print(f"Timer tick: {sec}")
            return True
        else:
            return False
        
    def dr_timer(self, sec):
        if time.time() - self.dr_time > sec:
            self.dr_time = time.time()
            # print(f"Timer tick: {sec}")
            return True
        else:
            return False
    
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
            
    def initiate(self):
        if not self.initiated:
            while not self.init_all_msgs():
                # print("initializing..")
                self.update_sensor_data()
                self.last_pos = self.RH.nav_pos
                self.last_hdg = self.RH.nav_hdg
            self.imu_time = time.time()
            self.dr_time = time.time()
            self.initiated = True
            self.initiate_q()
            

            # print("initiated")

    def update_sensor_data(self): # updates when pvt callbacked
        while self.RH.pvt_cb:
            self.RH.can_velocity_last = self.RH.can_velocity
            self.RH.corr_can_velocity_last = self.RH.corr_can_velocity
            self.RH.nav_hdg_last = self.RH.nav_hdg
            self.RH.nav_pos_last = self.RH.nav_pos
            self.RH.pvt_cb = False
        self.accel_integral = 0
        self.gyro_integral = 0

    def initiate_q(self):
        self.RH.q = self.euler_to_quaternion(np.deg2rad(self.RH.nav_roll), np.deg2rad(self.RH.nav_pitch), np.deg2rad(self.RH.nav_hdg))
        imu_hdg_tmp = np.rad2deg(np.arctan2(2.0*(self.RH.q[0]*self.RH.q[3] + self.RH.q[1]*self.RH.q[2]), 1.0 - 2.0*(self.RH.q[2]**2 + self.RH.q[3]**2)))
        # self.initial_offset = self.RH.nav_hdg - imu_hdg_tmp
        self.initial_offset = 0
        imu_hdg_tmp += self.initial_offset
        if self.RH.curved:
            constant_offset = 0
        else:
            constant_offset = 0 
        self.RH.imu_hdg = (imu_hdg_tmp + constant_offset)%360

    def calculate_dr_pos(self):
        dt = 0.05
        if self.RH.curved:
            offset = 0.008
            # print("curved")
        else:
            offset = 0.0
            # print("straight")

        # print(self.RH.can_steer)
        x_delta = (dt * self.RH.corr_can_velocity_last) * math.cos(math.radians(self.RH.nav_hdg))
        y_delta = (dt * self.RH.corr_can_velocity_last) * math.sin(math.radians(self.RH.nav_hdg))
        x_delta_wth_offset = (dt * self.RH.corr_can_velocity_last) * (math.cos(math.radians(self.RH.nav_hdg)) - offset*self.RH.can_steer*math.sin(math.radians(self.RH.nav_hdg)))
        y_delta_wth_offset = (dt * self.RH.corr_can_velocity_last) * (math.sin(math.radians(self.RH.nav_hdg)) + offset*self.RH.can_steer*math.cos(math.radians(self.RH.nav_hdg)))
        normalize_factor = (x_delta**2+y_delta**2)**0.5 / (x_delta_wth_offset**2+y_delta_wth_offset**2)**0.5
        x_delta_normalized = x_delta_wth_offset * normalize_factor
        y_delta_normalized = y_delta_wth_offset * normalize_factor
        self.dr_pos = [self.last_pos[0] + x_delta_normalized, self.last_pos[1] + y_delta_normalized]

    
    def calculate_dr_hdg(self):
        dt = 0.05
        delta_rad = math.radians(self.RH.can_steer_last)
        self.dr_hdg = self.last_hdg + (dt * math.degrees((self.RH.corr_can_velocity_last / 2.72) * math.tan(delta_rad)))
        
    
    def calculate_imu_hdg(self):
        pass

    def localization_sensor_health(self):
        if self.nav_hdg_invalid_cnt >= self.hdg_mct * 20 * 0.8:
            self.RH.nav_health_pub.publish(False)
            #TODO Restart RTK
        if self.nav_pos_invalid_cnt >= self.pos_mct * 20 * 0.8:
            self.RH.nav_health_pub.publish(False)
            #TODO Restart RTK

    def init_all_msgs(self):
        key1, key2, key3, key4 = False, False, False, False
        vehiclestates = [self.RH.corr_can_velocity_last, self.RH.corr_can_velocity, self.RH.can_velocity_last, self.RH.can_velocity, self.RH.can_steer, self.RH.can_steer_last]
        navatts = [self.last_hdg, self.RH.nav_hdg, self.RH.nav_pitch, self.RH.nav_roll]
        navpvts = [self.RH.nav_pos_last[0], self.RH.nav_pos[0], self.RH.nav_pos_last[1], self.RH.nav_pos[1]]
        imus = [self.RH.imu_angular_velocity, self.RH.imu_linear_acceleration, self.RH.imu_header]
        if not None in vehiclestates:
            key1 = True
        if not None in navatts:
            key2 = True
        if not None in navpvts:
            key3 = True
        if not None in imus:
            key4 = True

        if key1 and key2 and key3 and key4:
            return True

        # print(key1, key2, key3, key4)
        return False
    
    def print_pos_error(self):
        xerror = self.RH.nav_pos[0] - self.dr_pos[0]
        yerror = self.RH.nav_pos[1] - self.dr_pos[1]
        print(f"at speed {self.RH.corr_can_velocity*3.6:3.2f}, pos error: {(xerror**2+yerror**2)**0.5:2.2f} ||||\
    x: {xerror:.3f} \
    y: {yerror:.3f}")
        
    
    def print_hdg_error(self, target="DR"):
        if target == "DR":
            error = self.RH.nav_hdg - self.dr_hdg
            print(f"at speed {self.RH.corr_can_velocity*3.6:3.2f}, hdg error: {error:2.2f}")
        elif target == "IMU":
            error = self.RH.nav_hdg - self.RH.imu_hdg
            print(f"at speed {self.RH.corr_can_velocity*3.6:3.2f}, hdg error: {error:2.2f}")


    def update_last_pos(self):
        if self.RH.hAcc < 30:
            nav_pos_valid = True
        else:
            nav_pos_valid = False
            self.nav_pos_invalid_cnt += 1
        
        if None in [self.last_pos, self.dr_pos]:
            dr_pos_valid = False
        
        diff = ((self.dr_pos[0]-self.last_pos[0])**2 + (self.dr_pos[1]-self.last_pos[1])**2)**0.5
        if diff < 5:
            dr_pos_valid = True
        else:
            dr_pos_valid = False
        
        # if source == "NAV":
        if nav_pos_valid:
            self.last_pos = self.RH.nav_pos
        # elif source == "DR":
        elif dr_pos_valid:
            self.last_pos = self.dr_pos
        else:
            self.RH.nav_health_pub.publish(False)  # emergency stop
            #TODO Restart RTK

    def update_last_hdg(self):
        if self.RH.headAcc < 30000:
            nav_hdg_valid = True
        else:
            nav_hdg_valid = False
            self.nav_hdg_invalid_cnt += 1
        
        if None in [self.last_hdg, self.dr_hdg]:
            dr_hdg_valid = False
        val = abs(self.last_hdg - self.dr_hdg)
        diff = min(val, 360 - val)
        if diff < 5:
            dr_hdg_valid = True
        else:
            dr_hdg_valid = False
        
        if None in [self.last_hdg, self.RH.imu_hdg]:
            imu_hdg_valid = False
        val = abs(self.last_hdg - self.RH.imu_hdg)
        diff = min(val, 360 - val)
        if diff < 5:
            imu_hdg_valid = True
        else:
            imu_hdg_valid = False
        
        # if source == "NAV":
        if nav_hdg_valid:
            self.last_hdg = self.RH.nav_hdg
            self.initiate_q()
        # elif source == "DR":
        elif dr_hdg_valid:
            self.last_hdg = self.dr_hdg
        # elif source == "IMU":
        elif imu_hdg_valid:
            self.last_hdg = self.RH.imu_hdg
        else:
            self.RH.nav_health_pub.publish(False)  # emergency stop
            #TODO Restart RTK    
        
    def update_plot(self, target):
        nav_hdgs.append(self.RH.nav_hdg)
        nav_poss.append(self.RH.nav_pos)
        dr_poss.append(self.dr_pos)
        dr_hdgs.append(self.dr_hdg)
        imu_hdgs.append(self.RH.imu_hdg)



        self.ax.clear()
        
        for el in [nav_poss, dr_poss, nav_hdgs, dr_hdgs, imu_hdgs]:
            if len(el) > 100:
                el.pop(0)

        if target == "POS":        
            self.ax.plot([el[0] for el in nav_poss], [el[1] for el in nav_poss], label='NAV')
            self.ax.plot([el[0] for el in dr_poss], [el[1] for el in dr_poss], label='DR')
        elif target == "HDG_DR":
            self.ax.plot(nav_hdgs, label='NAV')
            self.ax.plot(dr_hdgs, label='DR')
        elif target == "HDG_IMU":
            self.ax.plot(nav_hdgs, label='NAV')
            self.ax.plot(imu_hdgs, label='IMU')

        

        plt.grid()
        plt.draw()
        plt.legend()
        plt.pause(0.00001)

    def run(self): 
        
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.initiate() # initiate module
            self.update_sensor_data() # update CAN, NavATT, NavPVT sensor datas 
            self.calculate_dr_pos() # estimate current pos from last pos & sensor datas
            self.calculate_dr_hdg() # estimate current hdg from last hdg & sensor datas
            self.calculate_imu_hdg()
            ### self.accel, gyro_integral = 0
            self.update_last_pos() # update last_pos variable (choose: NAV, DR)
            self.update_last_hdg() # update last_hdg variable (choose: NAV, DR, IMU)
            # self.print_pos_error() # print pos error in terminal
            # self.print_hdg_error(target="DR") # print hdg error in terminal
            # self.update_plot(target="HDG_DR") # plot (choose: POS, HDG_DR, HDG_IMU)
            self.localization_sensor_health()
            if self.last_hdg is not None and self.last_pos is not None:
                self.RH.publish(self.last_hdg, self.last_pos)
            rate.sleep()



def signal_handler(sig, frame):
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    localization = Localization()
    localization.run()

if __name__ == "__main__":
    main()

# if __name__ == "__main__":
#     rospy.init_node("dr_simul")
#     dr = Localization()
#     rate = rospy.Rate(20)
#     while not rospy.is_shutdown():
#         dr.run()
#         rate.sleep()

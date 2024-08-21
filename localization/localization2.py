import math
import rospy
from ublox_msgs.msg import NavATT, NavPVT
from drive_msgs.msg import VehicleState, CANOutput, LaneData
from pyproj import Proj, Transformer
import matplotlib.pyplot as plt

import time


nav_poss = []
dr_poss = []
nav_hdgs = []
dr_hdgs = []

class DR_BICYCLE:
    def __init__(self):
        rospy.init_node("dr_simul")
        rospy.Subscriber("/VehicleState", VehicleState, self.VS_cb)
        self.corr_can_velocity_last = None
        self.corr_can_velocity = None
        self.can_velocity_last = None
        self.can_velocity = None
        rospy.Subscriber("/CANOutput", CANOutput, self.CO_cb)
        self.can_steer_last = None
        self.can_steer = None
        rospy.Subscriber("/ublox/navatt", NavATT, self.ATT_cb)
        self.nav_hdg_last = None
        self.nav_hdg = None
        rospy.Subscriber("/ublox/navpvt", NavPVT, self.PVT_cb)
        base_lla = [35.65492524, 128.39351431, 7] # KIAPI_Racing base
        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=base_lla[0], lon_0=base_lla[1], h_0=base_lla[2])
        self.transformer = Transformer.from_proj(proj_wgs84, proj_enu)
        self.nav_x_last = None
        self.nav_y_last = None
        self.nav_pos_last = [None, None]
        self.nav_x = None
        self.nav_y = None
        self.nav_pos = [None, None]
        self.gspeed = None
        self.pvt_cb = False
        rospy.Subscriber("/LaneData", LaneData, self.LD_cb)
        self.curve_list = ['1', '7', '8', '9', '10', '11', '15', '16', '17', '21', '22', '23',
                           '24', '25', '26', '27', '36', '37', '38', '39', '43', '44', '54', '59',
                           '60', '61', '62', '63', '68', '69', '70', '72', '73', '78', '79', '80']
        self.curved = False

        self.initiated = False

        self.last_pos = None
        self.last_hdg = None

        self.dr_pos = [None, None]
        self.dr_hdg = None

        # Initialize plot
        self.fig, self.ax = plt.subplots()
        self.nav_plot, = self.ax.plot([], [], 'bo-', label='NAV')
        self.dr_plot, = self.ax.plot([], [], 'ro-', label='DR')
        self.ax.legend()
        # self.ax.set_xlim(-100, 100)
        # self.ax.set_ylim(-100, 100)
        self.ax.axis("equal")

    def timer(self, sec):
        if time.time() - self.last_time > sec:
            self.last_time = time.time()
            print(f"Timer tik: {sec}")
            return True
        else:
            return False
        
    def LD_cb(self, msg):
        if msg.currentLane.id.data in self.curve_list:
            self.curved = True
        else:
            self.curved = False
    def VS_cb(self, msg):
        self.can_velocity = msg.velocity.data # [m/s]
        # self.params = [-8.38357609e-03, 2.37367164e-02, -1.59672708e-04, 1.53623118e-06]
        self.s_params = [-1.69519446e-01, 3.14832448e-02, -2.42469118e-04, 1.68413777e-06]
        self.c_params = [-3.04750083e-01, 4.43420297e-02, -6.07069742e-04, 4.46079605e-06]

        if self.curved:
            self.params = self.c_params
        else:
            self.params = self.s_params

        self.corr_can_velocity = (self.can_velocity*3.6 \
                            + self.params[0] + self.params[1]*(self.can_velocity*3.6) \
                            + self.params[2]*((self.can_velocity*3.6)**2) \
                            + self.params[3]*((self.can_velocity*3.6)**3))/3.6 # [m/s]  
        
    
    def CO_cb(self, msg):
        self.can_steer_last = self.can_steer
        handle_ang = float(msg.StrAng.data)
        steer_scale_factor = 32.2/450
        self.can_steer = handle_ang*steer_scale_factor

    def ATT_cb(self, msg):
        self.nav_hdg = -(msg.heading*1e-5 - 90)%360 # [deg]

        nav_hdgs.append(self.nav_hdg)

    def PVT_cb(self, msg):
        lat = msg.lat*1e-7
        lon = msg.lon*1e-7
        x, y, _= self.transformer.transform(lon, lat, 7)
        self.nav_pos = [x, y]
        self.nav_x = x
        self.nav_y = y
        self.pvt_cb = True
        self.gspeed = msg.gSpeed

        nav_poss.append([self.nav_x, self.nav_y])
    
    def initiate(self):
        if not self.initiated:
            while not self.init_all_msgs():
                print("initializing..")
                self.update_sensor_data()
                self.last_pos = [self.nav_x, self.nav_y]
                self.last_hdg = self.nav_hdg
            self.last_time = time.time()
            self.initiated = True
            print("\n\n\n\n\n\n\n\ninitiated\n\n\n\n\n\n\n\n\n")

    def update_sensor_data(self): # updates when pvt callbacked
        while self.pvt_cb:
            self.can_velocity_last = self.can_velocity
            self.corr_can_velocity_last = self.corr_can_velocity
            self.nav_hdg_last = self.nav_hdg
            self.nav_x_last = self.nav_x
            self.nav_y_last = self.nav_y
            self.pvt_cb = False

    
    def calculate_dr_pos(self):
        dt = 0.05
        if self.curved:
            offset = 0.008
        else:
            offset = 0.0

        x_delta = (dt * self.corr_can_velocity_last) * math.cos(math.radians(self.nav_hdg))
        y_delta = (dt * self.corr_can_velocity_last) * math.sin(math.radians(self.nav_hdg))
        x_delta_wth_offset = (dt * self.corr_can_velocity_last) * (math.cos(math.radians(self.nav_hdg)) - offset*self.can_steer*math.sin(math.radians(self.nav_hdg)))
        y_delta_wth_offset = (dt * self.corr_can_velocity_last) * (math.sin(math.radians(self.nav_hdg)) + offset*self.can_steer*math.cos(math.radians(self.nav_hdg)))
        normalize_factor = (x_delta**2+y_delta**2)**0.5 / (x_delta_wth_offset**2+y_delta_wth_offset**2)**0.5
        x_delta_normalized = x_delta_wth_offset * normalize_factor
        y_delta_normalized = y_delta_wth_offset * normalize_factor
        self.dr_pos = [self.last_pos[0] + x_delta_normalized, self.last_pos[1] + y_delta_normalized]

        dr_poss.append(self.dr_pos)
    
    def calculate_dr_hdg(self):
        dt = 0.05
        delta_rad = math.radians(self.can_steer_last)
        self.dr_hdg = self.last_hdg + (dt * math.degrees((self.corr_can_velocity_last / 2.72) * math.tan(delta_rad)))
        dr_hdgs.append(self.dr_hdg)

    def all_msgs(self):
        key1, key2, key3, key4 = False, False, False, False
        candatas = [self.corr_can_velocity_last, self.corr_can_velocity, self.can_velocity_last, self.can_velocity, self.can_steer, self.can_steer_last]
        navatts = [self.nav_hdg_last, self.nav_hdg]
        navpvts = [self.nav_x_last, self.nav_x, self.nav_y_last, self.nav_y]
        additionals = [self.last_pos, self.last_hdg]
        if not None in candatas:
            key1 = True
        if not None in navatts:
            key2 = True
        if not None in navpvts:
            key3 = True
        if not None in additionals:
            key4 = True

        if key1 and key2 and key3 and key4:
            return True
        return False
    
    def init_all_msgs(self):
        key1, key2, key3 = False, False, False
        vehiclestates = [self.corr_can_velocity_last, self.corr_can_velocity, self.can_velocity_last, self.can_velocity, self.can_steer]
        navatts = [self.nav_hdg_last, self.nav_hdg]
        navpvts = [self.nav_x_last, self.nav_x, self.nav_y_last, self.nav_y]
        if not None in vehiclestates:
            key1 = True
        if not None in navatts:
            key2 = True
        if not None in navpvts:
            key3 = True

        if key1 and key2 and key3:
            return True
        return False
    
    def print_pos_error(self):
        xerror = self.nav_x - self.dr_pos[0]
        yerror = self.nav_y - self.dr_pos[1]
        print(f"at speed {self.corr_can_velocity*3.6:3.2f}, pos error: {(xerror**2+yerror**2)**0.5:2.2f} ||||\
    x: {xerror:.3f} \
    y: {yerror:.3f}")
        
    
    def print_hdg_error(self):
        error = self.nav_hdg - self.dr_hdg
        print(f"at speed {self.corr_can_velocity*3.6:3.2f}, hdg error: {error:2.2f}")


    def update_last_pos(self, source="NAV"):
        if source == "NAV":
            self.last_pos = [self.nav_x_last, self.nav_y_last]
        elif source == "DR":
            if self.timer(5):
                print("------------------[pos] update to nav----------------")
                self.last_pos = [self.nav_x, self.nav_y]
            else:
                self.last_pos = self.dr_pos

    def update_last_hdg(self, source="NAV"):
        if source == "NAV":
            self.last_hdg = self.nav_hdg
        elif source == "DR":
            if self.timer(5):
                print("------------------[hdg] update to nav----------------")
                self.last_hdg = self.nav_hdg
            else:
                self.last_hdg = self.dr_hdg
        

    def update_plot(self, target):
        self.ax.clear()
        
        for el in [nav_poss, dr_poss, nav_hdgs, dr_hdgs]:
            if len(el) > 100:
                el.pop(0)

        if target == "POS":        
            self.ax.plot([el[0] for el in nav_poss], [el[1] for el in nav_poss], label='NAV')
            self.ax.plot([el[0] for el in dr_poss], [el[1] for el in dr_poss], label='DR')
        elif target == "HDG":
            self.ax.plot(nav_hdgs, label='NAV')
            self.ax.plot(dr_hdgs, label='DR')

        

        plt.grid()
        plt.draw()
        plt.legend()
        plt.pause(0.00001)

    def run(self):
        self.initiate() # initiate module
        self.update_sensor_data() # update CAN, NavATT, NavPVT sensor datas 
        self.calculate_dr_pos() # estimate current pos from last pos & sensor datas
        self.calculate_dr_hdg() # estimate current hdg from last hdg & sensor datas
        self.update_last_pos(source="NAV") # update last_pos variable (either DR or NAV)
        self.update_last_hdg(source="DR") # update last_hdg variable (either DR or NAV)
        self.print_pos_error() # print pos error in terminal
        # self.print_hdg_error() # print hdg error in terminal
        # self.update_plot(target="HDG") # plot (either POS or HDG)

        ## 실행 -> 여기서 아래 코드 참고하여 모듈 추가 -> valid 모듈 추가


if __name__ == "__main__":
    dr = DR_BICYCLE()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        dr.run()
        rate.sleep()



import rospy
import sys
import signal

from ros_handler import ROSHandler
# from dr_bicycle import DR_BICYCLE
# from imu_heading import ImuHeading

def signal_handler(sig, frame):
    sys.exit(0)

class BestLocalization:
    def __init__(self):
        self.initiated = False
        self.RH = ROSHandler()
        self.DR = DR_BICYCLE()

        self.nav_hdg = None
        self.nav_pos = None
        self.dr_hdg = None
        self.dr_pos = [None, None]
        self.imu_hdg = None

        self.best_hdg = None
        self.best_pos = [None, None]
        
        self.nav_hdg_last = None
        self.nav_pos_last = [None, None]
        self.dr_hdg_last = None
        self.dr_pos_last = [None, None]
        self.imu_hdg_last = None

        self.best_hdg_last = None
        self.best_pos_last = [None, None]

        self.nav_cw_cnt = 0
        self.imu_cw_cnt = 0
        self.dr_cw_cnt = 0
        self.p_nav_hdg = None
        self.p_imu_hdg = None
        self.p_dr_hdg = None

        self.nav_hdg_valid = True
        self.nav_pos_valid = True

        self.llh = [None, None]
        
    def update_sensors1(self): # sensor values update
        self.nav_hdg_last = self.DR.nav_hdg_last
        self.nav_pos_last = [self.DR.nav_x_last, self.DR.nav_y_last]

        self.nav_hdg = self.RH.nav_hdg
        self.nav_pos = self.RH.nav_pos

    def update_sensors2(self): # calculated values update
        self.dr_hdg_last = self.dr_hdg
        self.dr_pos_last = self.dr_pos
        self.imu_hdg_last = self.imu_hdg
        self.best_hdg_last = self.best_hdg
        self.best_pos_last = self.best_pos

        self.dr_hdg = self.DR.dr_hdg
        self.dr_pos = self.DR.dr_pos
        self.imu_hdg = self.IH.imu_corr_heading
    
    def valid_hdg(self, hdg_last, hdg_now, threshold): # not used yet, variable 'diff' needs field test
        if None in [hdg_last, hdg_now]:
            return False
        
        val = abs(hdg_last - hdg_now)
        diff = min(val, 360 - val)
        result = diff < threshold
        
        return result

    def valid_pos(self, pos_last, pos_now, threshold): # not used yet, variable 'diff' needs field test
        if None in [pos_last[0], pos_now[0]]:
            return False
        
        diff = ((pos_now[0]-pos_last[0])**2 + (pos_now[1]-pos_last[1])**2)**0.5
        result = diff < threshold

        return result
        
    def hdg_postprocess(self):
        if self.nav_hdg - self.nav_hdg_last < -355:
            self.nav_cw_cnt -= 1
        if self.nav_hdg - self.nav_hdg_last > 355:
            self.nav_cw_cnt += 1
        self.p_nav_hdg = self.nav_hdg - self.nav_cw_cnt * 360

        if self.imu_hdg - self.imu_hdg_last < -355:
            self.imu_cw_cnt -= 1
        if self.imu_hdg - self.imu_hdg_last > 355:
            self.imu_cw_cnt += 1
        self.p_imu_hdg = self.imu_hdg - self.imu_cw_cnt * 360

        if self.dr_hdg - self.dr_hdg_last < -355:
            self.dr_cw_cnt -= 1
        if self.dr_hdg - self.dr_hdg_last > 355:
            self.dr_cw_cnt += 1
        self.p_dr_hdg = self.dr_hdg - self.dr_cw_cnt * 360

    def integrate_hdg(self, hz):
        self.nav_hdg_valid = self.valid_hdg(self.best_hdg_last, self.nav_hdg, 20)
        imu_valid = self.valid_hdg(self.best_hdg_last, self.imu_hdg, 5)
        dr_valid = self.valid_hdg(self.best_hdg_last, self.dr_hdg, 5)

        
        if self.nav_hdg_valid:
            self.best_hdg = self.p_nav_hdg
            print_result = "navatt_hdg"
        else:
            self.best_hdg = self.p_dr_hdg
            print_result = "ddreck_hdg"

        return print_result, [self.nav_hdg_valid, imu_valid, dr_valid]

    def integrate_position(self, hz):
        self.nav_pos_valid = self.valid_pos(self.best_pos_last, self.nav_pos, 30)
        dr_valid = self.valid_pos(self.best_pos_last, self.dr_pos, 5)

        if self.nav_pos_valid:
            self.best_pos = self.nav_pos
            print_result = "navpvt_position"
        elif not self.nav_pos_valid and dr_valid:
            self.best_pos = self.dr_pos
            print_result = "deadrk_position"
        else:
            print_result = "all position Dd"

        return print_result, [self.nav_pos_valid, dr_valid]
    
    def initiate(self):
        if not self.initiated:
            key1, key2 = False, False
            # while self.nav_hdg_last is None:
            while 1:
                self.best_hdg = self.nav_hdg
                self.best_pos = self.nav_pos
                self.update_sensors1()
                self.update_sensors2()
                
                if self.best_hdg is not None and self.best_pos[0] is not None:
                    key1 = True
                if self.best_hdg_last is not None and self.best_pos_last[0] is not None:
                    key2 = True
                if key1 and key2:
                    self.initiated = True
                    break

    def run(self):
        rate = rospy.Rate(20)
        str6 = "GOOD NAV"

        while not rospy.is_shutdown():
            self.initiate()

            self.update_sensors1()
            self.DR.run(self.best_hdg_last, self.best_pos_last)
            self.IH.run(self.best_hdg, self.nav_hdg_valid)
            self.update_sensors2()

            hdg_source, position_source = "not determined", "not determined"

            if None not in [self.nav_hdg_last, self.imu_hdg_last, self.dr_hdg_last]:
                self.hdg_postprocess()
                hdg_source, hdg_validity = self.integrate_hdg(20)      

            if self.nav_pos_last[0] is not None and self.dr_pos_last[0] is not None:
                position_source, position_validity = self.integrate_position(20)

            if None not in [self.best_hdg, self.best_pos[0]]:
                self.RH.publish(self.best_hdg, self.best_pos)
            
            if position_source == "deadrk_position":
                print("dead reckoning")
            
            try:
                str1 = "-------------------------------------------\n"
                str2 = f"nav:{hdg_validity[0]} | BEST HDG         | nav:{position_validity[0]} | BEST POS\n"
                str3 = f"imu:{hdg_validity[1]} | {self.best_hdg%360:3.4f}         |          | {self.best_pos[0]:3.4f}\n"
                str4 = f"dr :{hdg_validity[2]} |                  | dr :{position_validity[1]} | {self.best_pos[1]:3.4f}\n"
                str5 = f"HDG SOURCE : {hdg_source} | POS SOURCE : {position_source}\n"
                if hdg_source == "imu+dr_hdg" or position_source == "deadrk_position":
                    str6 = f"NOT NAV"
                # print(str1+str2+str3+str4+str5+str6)
            except:
                pass

            rate.sleep()

# def main():
#     signal.signal(signal.SIGINT, signal_handler)
#     best_localization = BestLocalization()
#     best_localization.run()

# if __name__ == "__main__":
#     main()

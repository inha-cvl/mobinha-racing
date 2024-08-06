import rospy
import sys
import signal

from ros_handler import ROSHandler
from dr_bicycle import DR_BICYCLE
from imu_heading import ImuHeading

from pyproj import Proj, Transformer
import numpy as np

def signal_handler(sig, frame):
    sys.exit(0)

class BestLocalization:
    def __init__(self):
        self.initiated = False
        self.RH = ROSHandler()
        self.DR = DR_BICYCLE(self.RH)
        self.IH = ImuHeading(self.RH)

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

        self.nav_hdg_valid = True
        self.nav_pos_valid = True

        self.llh = [None, None]
        
    def update_sensors1(self):
        self.nav_heading_last = self.RH.nav_heading_last
        self.nav_pos_last = self.RH.nav_pos_last

        self.nav_heading = self.RH.nav_heading
        self.nav_pos = self.RH.nav_pos

    def update_sensors2(self):
        self.dr_heading_last = self.dr_heading
        self.dr_pos_last = self.dr_pos
        self.imu_heading_last = self.imu_heading
        self.best_heading_last = self.best_heading
        self.best_pos_last = self.best_pos

        self.dr_heading = self.DR.dr_heading
        self.dr_pos = self.DR.dr_pos
        self.imu_heading = self.IH.imu_corr_heading
    
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
        self.nav_hdg_valid = self.valid_hdg(self.best_heading_last, self.nav_heading, 20)
        imu_valid = self.valid_hdg(self.best_heading_last, self.imu_heading, 5)
        dr_valid = self.valid_hdg(self.best_heading_last, self.dr_heading, 5)

        imu_weight = 0.5
        dr_weight = 0.0
        if not imu_valid:
            imu_weight = 0
        if not dr_valid:
            dr_weight = 0
        
        if self.nav_hdg_valid:
            self.best_heading = self.p_nav_heading
            print_result = "navatt_heading"
        else:
            self.best_heading = (self.p_imu_heading*imu_weight + self.p_dr_heading*dr_weight)/(imu_weight+dr_weight)
            print_result = "imu+dr_heading"

        return print_result, [self.nav_hdg_valid, imu_valid, dr_valid]

    def integrate_position(self, hz):
        self.nav_pos_valid = self.valid_pos(self.best_pos_last, self.nav_pos, 30)
        dr_valid = self.valid_pos(self.best_pos_last, self.dr_pos, 5)

        if self.nav_pos_valid:
            self.best_pos = self.nav_pos
            #print("using navpvt position")
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
            # while self.nav_heading_last is None:
            while 1:
                self.best_heading = self.nav_heading
                self.best_pos = self.nav_pos
                self.update_sensors1()
                self.update_sensors2()
                
                if self.best_heading is not None and self.best_pos[0] is not None:
                    key1 = True
                if self.best_heading_last is not None and self.best_pos_last[0] is not None:
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
            self.DR.run(self.best_heading_last, self.best_pos_last)
            self.IH.run(self.best_heading, self.nav_hdg_valid)
            self.update_sensors2()

            heading_source, position_source = "not determined", "not determined"

            if None not in [self.nav_heading_last, self.imu_heading_last, self.dr_heading_last]:
                self.heading_postprocess()
                heading_source, heading_validity = self.integrate_heading(20)      

            if self.nav_pos_last[0] is not None and self.dr_pos_last[0] is not None:
                position_source, position_validity = self.integrate_position(20)

            if None not in [self.best_heading, self.best_pos[0]]:
                self.RH.publish(self.best_heading, self.best_pos)
            
            if position_source == "deadrk_position":
                print("dead reckoning")
            
            try:
                str1 = "-------------------------------------------\n"
                str2 = f"nav:{heading_validity[0]} | BEST HDG         | nav:{position_validity[0]} | BEST POS\n"
                str3 = f"imu:{heading_validity[1]} | {self.best_heading%360:3.4f}         |          | {self.best_pos[0]:3.4f}\n"
                str4 = f"dr :{heading_validity[2]} |                  | dr :{position_validity[1]} | {self.best_pos[1]:3.4f}\n"
                str5 = f"HDG SOURCE : {heading_source} | POS SOURCE : {position_source}\n"
                if heading_source == "imu+dr_heading" or position_source == "deadrk_position":
                    str6 = f"NOT NAV"
                print(str1+str2+str3+str4+str5+str6)
            except:
                pass

            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    best_localization = BestLocalization()
    best_localization.run()

if __name__ == "__main__":
    main()

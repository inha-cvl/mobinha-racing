import rospy
import sys
import signal

from ros_handler import ROSHandler
from dr_bicycle import DR_BICYCLE
from imu_heading import ImuHeading

def signal_handler(sig, frame):
    sys.exit(0)

class BestLocalization:
    def __init__(self):
        self.initialize = True
        self.RH = ROSHandler(map)
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

def main():
    signal.signal(signal.SIGINT, signal_handler)
    best_localization = BestLocalization()
    best_localization.run()

if __name__ == "__main__":
    main()

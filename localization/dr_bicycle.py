import math

from pyproj import Proj, Transformer

class DR_BICYCLE:
    def __init__(self, RH):
        self.RH = RH

        self.wheelbase = 2.72

        # dt
        self.dt = None # [sec]

        # result
        self.dr_pos = [None, None]
        self.dr_hdg = None   

    def check_msg_valid(self):
        nav_checklist = [self.RH.nav_header, self.RH.nav_hdg, self.RH.nav_pos] # excluded self.nav_velocity 
        nav_last_checklist = [self.RH.nav_header_last, self.RH.nav_hdg_last, self.RH.nav_pos_last] # excluded self.nav_velocity_last
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

    def calculate_nextpos(self, best_hdg_last, best_pos_last):
        delta_rad = math.radians(self.RH.can_steer_last)

        theta_rad = math.radians(self.RH.nav_hdg_last) if best_hdg_last is None else math.radians(best_hdg_last)
        start_pos = self.RH.nav_pos_last if best_pos_last[0] is None else best_pos_last

        if self.curved:
            offset = 0.008
        else:
            offset = 0.0
        x_delta = (self.dt * self.RH.corr_can_velocity_last) * math.cos(theta_rad)
        y_delta = (self.dt * self.RH.corr_can_velocity_last) * math.sin(theta_rad)
        x_delta_wth_offset = (self.dt * self.RH.corr_can_velocity_last) * (math.cos(theta_rad) - offset*self.can_steer*math.sin(theta_rad))
        y_delta_wth_offset = (self.dt * self.RH.corr_can_velocity_last) * (math.sin(theta_rad) + offset*self.can_steer*math.cos(theta_rad))
        normalize_factor = (x_delta**2+y_delta**2)**0.5 / (x_delta_wth_offset**2+y_delta_wth_offset**2)**0.5
        x_delta_normalized = x_delta_wth_offset * normalize_factor
        y_delta_normalized = y_delta_wth_offset * normalize_factor
        self.dr_pos = [start_pos[0] + x_delta_normalized, start_pos[1] + y_delta_normalized]


        dr_x = start_pos[0] + x_delta
        dr_y = start_pos[1] + y_delta
        self.dr_pos = [dr_x, dr_y]

        self.dr_hdg = math.degrees(theta_rad) + (self.dt * math.degrees((self.RH.corr_can_velocity_last / 2.72) * math.tan(delta_rad)))
        
        ## original(0821)
        # x_delta = (self.dt * self.RH.corr_can_velocity_last) * math.cos(theta_rad)
        # y_delta = (self.dt * self.RH.corr_can_velocity_last) * math.sin(theta_rad)
        # dr_x = start_pos[0] + x_delta
        # dr_y = start_pos[1] + y_delta
        # self.dr_pos = [dr_x, dr_y]
        # self.dr_hdg = best_hdg_last + (self.dt * math.degrees((self.RH.corr_can_velocity_last / self.wheelbase) * math.tan(delta_rad)))

    
    def run(self, best_hdg_last, best_pos_last): 
        if self.check_msg_valid():
            self.calculate_dt()
            self.calculate_nextpos(best_hdg_last, best_pos_last) 
import math

from pyproj import Proj, Transformer

from ros_handler import ROSHandler

class DR_BICYCLE:
    def __init__(self):
        self.RH = ROSHandler(map)

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
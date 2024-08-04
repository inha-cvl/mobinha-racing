import rospy, math
from drive_msgs.msg import CANOutput
from ublox_msgs.msg import NavPVT
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

from pyproj import Proj, Transformer

from datetime import datetime

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
        if self.check_msg_valid():
            self.calculate_dt()
            self.calculate_nextpos() 

# if __name__ == "__main__":
#     rospy.init_node('dr_model')
#     model = DR_BICYCLE()
#     while not rospy.is_shutdown():
#         model.run()
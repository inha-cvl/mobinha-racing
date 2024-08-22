import math
import rospy
from ublox_msgs.msg import NavATT, NavPVT
from drive_msgs.msg import VehicleState, CANOutput
from std_msgs.msg import Bool
from pyproj import Proj, Transformer
import matplotlib.pyplot as plt

nav_poss = []
dr_poss = []

class DR_BICYCLE:
    def __init__(self):
        # rospy.init_node("dr_simul")
        rospy.Subscriber("/VehicleState", VehicleState, self.VS_cb)
        self.corr_can_velocity_last = None
        self.corr_can_velocity = None
        self.can_velocity_last = None
        self.can_velocity = None
        rospy.Subscriber('/CANOutput', CANOutput, self.CO_cb)
        self.can_steer = None
        self.can_steer_last = None
        rospy.Subscriber("/ublox/navatt", NavATT, self.ATT_cb)
        self.nav_heading_last = None
        self.nav_heading = None
        rospy.Subscriber("/ublox/navpvt", NavPVT, self.PVT_cb)
        base_lla = [35.65492524, 128.39351431, 7] # KIAPI_Racing base
        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=base_lla[0], lon_0=base_lla[1], h_0=base_lla[2])
        self.transformer = Transformer.from_proj(proj_wgs84, proj_enu)
        self.nav_x_last = None
        self.nav_y_last = None
        self.nav_x = None
        self.nav_y = None
        self.gspeed = None
        self.pvt_cb = False

        self.initiated = False

        self.last_pos = None

        self.dr_pos = None
        self.dr_heading = None
        # Initialize plot
        self.fig, self.ax = plt.subplots()
        self.nav_plot, = self.ax.plot([], [], 'bo-', label='NAV')
        self.dr_plot, = self.ax.plot([], [], 'ro-', label='DR')
        self.ax.legend()
        self.ax.set_xlim(-100, 100)
        self.ax.set_ylim(-100, 100)

        rospy.Subscriber("/position_hack", Bool, self.pos_hack_cb)
        self.hacked = False
    
    def pos_hack_cb(self, msg):
        self.hacked = msg.data

    def VS_cb(self, msg):
        self.can_velocity = msg.velocity.data # [m/s]
        self.params = [-8.38357609e-03, 2.37367164e-02, -1.59672708e-04, 1.53623118e-06]
        # self.params = [-3.04750083e-01, 4.43420297e-02, -6.07069742e-04, 4.46079605e-06] # curve

        self.corr_can_velocity = (self.can_velocity*3.6 \
                            + self.params[0] + self.params[1]*(self.can_velocity*3.6) \
                            + self.params[2]*((self.can_velocity*3.6)**2) \
                                + self.params[3]*((self.can_velocity*3.6)**3))/3.6 # [m/s]

    def ATT_cb(self, msg):
        self.nav_heading = -(msg.heading*1e-5 - 90)%360 # [deg]

    def PVT_cb(self, msg):
        lat = msg.lat*1e-7
        lon = msg.lon*1e-7
        x, y, _= self.transformer.transform(lon, lat, 7)
        self.nav_x = x
        self.nav_y = y
        self.pvt_cb = True
        self.gspeed = msg.gSpeed

        nav_poss.append([self.nav_x, self.nav_y])

    def CO_cb(self, msg):
        handle_ang = float(msg.StrAng.data)
        steer_scale_factor = 32.2/450
        self.can_steer = handle_ang * steer_scale_factor
    
    def initiate(self):
        if not self.initiated:
            while not self.init_all_msgs():
                self.update_sensor_data()
                self.last_pos = [self.nav_x, self.nav_y]

    def update_sensor_data(self): # updates when pvt callbacked
        # while self.pvt_cb:
        self.can_velocity_last = self.can_velocity
        self.corr_can_velocity_last = self.corr_can_velocity
        self.nav_heading_last = self.nav_heading
        self.nav_x_last = self.nav_x
        self.nav_y_last = self.nav_y
        self.pvt_cb = False

    
    def calculate_dr_pos(self):
        dt = 0.05
        offset = 0.008
        x_delta = (dt * self.corr_can_velocity_last) * (math.cos(math.radians(self.nav_heading)) - offset * self.can_steer * math.sin(math.radians(self.nav_heading)))
        y_delta = (dt * self.corr_can_velocity_last) * (math.sin(math.radians(self.nav_heading)) + offset * self.can_steer * math.cos(math.radians(self.nav_heading)))
        self.dr_pos = [self.last_pos[0] + x_delta, self.last_pos[1] + y_delta]

        dr_poss.append(self.dr_pos)

    def calculate_dr_heading(self):
        self.dr_heading = 0

    def all_msgs(self):
        key1, key2, key3, key4 = False, False, False, False
        vehiclestates = [self.corr_can_velocity_last, self.corr_can_velocity, self.can_velocity_last, self.can_velocity]
        navatts = [self.nav_heading_last, self.nav_heading]
        navpvts = [self.nav_x_last, self.nav_x, self.nav_y_last, self.nav_y]
        additionals = [self.last_pos]
        if not None in vehiclestates:
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
        navatts = [self.nav_heading_last, self.nav_heading]
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
        error = (xerror**2 + yerror**2)**0.5
        # print(f"pos error: {error:.3f}")
            #   x: {xerror:.3f} \
            #   y: {yerror:.3f}")


    def update_last_pos(self):
        if self.hacked:
            self.last_pos = self.dr_pos
            # print("using DR pos")
        else:
            self.last_pos = [self.nav_x_last, self.nav_y_last]
            # print("using Nav pos")

    def update_plot(self):
        self.ax.clear()
        
        if len(nav_poss) > 50:
            nav_poss.pop(0)
        if len(dr_poss) > 50:
            dr_poss.pop(0)
        self.ax.plot([el[0] for el in nav_poss], [el[1] for el in nav_poss], label='nav')
        self.ax.plot([el[0] for el in dr_poss], [el[1] for el in dr_poss], label='dr')

        plt.grid()
        plt.draw()
        plt.legend()
        plt.pause(0.01)

    def run(self):
        self.initiate()
        self.update_sensor_data()
        self.calculate_dr_pos()
        self.update_last_pos()
        # self.update_plot()
        self.print_pos_error()


if __name__ == "__main__":
    dr = DR_BICYCLE()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        dr.run()
        rate.sleep()

import rospy

from drive_msgs.msg import *
from ublox_msgs.msg import NavATT, NavPVT
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Int8
from pyproj import Proj, Transformer
from ahrs.filters import Madgwick
import numpy as np

class ROSHandler():
    def __init__(self):
        rospy.init_node('localization', anonymous=False)
        self.set_values()
        self.set_protocol()
        self.set_params()

    def set_values(self):
        self.base_lla = None
        self.transformer = None
        self.local_pose = [0,0]   
        self.curr_lane_id = None
        
        self.nav_header = None
        self.nav_pos = [None, None]
        self.llh = [None, None]
        self.nav_hdg = None
        self.nav_roll = None
        self.nav_pitch = None

        self.nav_header_last = None
        self.nav_pos_last = [None, None]
        self.nav_hdg_last = None

        self.imu_header = None
        self.imu_angular_velocity = None
        self.imu_linear_acceleration = None

        self.can_velocity = None
        self.can_steer = None
        self.corr_can_velocity = None

        self.can_velocity_last = None
        self.can_steer_last = None
        self.corr_can_velocity_last = None

        self.laneNumber = None

        # tmp
        self.hdg_hack = False
        self.pos_hack = False
        self.nav_velocity = 0

        self.hdg_invalid_cnt = 0
        self.pos_invalid_cnt = 0

        # change
        self.curve_list = ['1', '7', '8', '9', '10', '11', '15', '16', '17', '21', '22', '23', '24', 
                           '25', '26', '27', '36', '37', '38', '39', '43', '44', '45', '46', '47', '54', 
                           '59', '60', '61', '62', '63', '68', '69', '70', '72', '73', '78', '79', '80']
        self.curved = False

        self.pvt_cb = False
        self.gspeed = None
        self.hAcc = None
        self.headAcc = None

        self.accel = None
        self.gyro = None
        self.accel_integral = 0
        self.gyro_integral = 0
        self.madgwick = Madgwick()
        self.initial_offset = 0
        self.q = None
        self.imu_hdg = None

    def set_protocol(self):
        rospy.Subscriber('/ublox/navatt', NavATT, self.navatt_cb)
        rospy.Subscriber('/ublox/navpvt', NavPVT, self.navpvt_cb)
        rospy.Subscriber('/ublox/imu_meas', Imu, self.imu_cb)
        rospy.Subscriber('/ublox/fix', NavSatFix, self.fix_cb)
        rospy.Subscriber('/CANOutput', CANOutput, self.canoutput_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        rospy.Subscriber('/LaneData', LaneData, self.lanedata_cb)
        
        # tmp
        self.pub_hdg = rospy.Subscriber("/heading_hack", Bool, self.hdg_cb)
        self.pub_pos = rospy.Subscriber("/position_hack", Bool, self.pos_cb)

        self.best_pose_pub = rospy.Publisher('/best/pose', Pose, queue_size=1)
        ## Sensot health
        self.nav_health_pub = rospy.Publisher('/nav_health', Int8, queue_size=1)

    def hdg_cb(self, msg):
        self.hdg_hack = msg.data
    def pos_cb(self, msg):
        self.pos_hack = msg.data

    def set_params(self):
        self.steer_scale_factor = 32.2/450 # 36.2/500  # need to change

        self.s_params = [-1.69519446e-01, 3.14832448e-02, -2.42469118e-04, 1.68413777e-06]
        self.c_params = [-3.04750083e-01, 4.43420297e-02, -6.07069742e-04, 4.46079605e-06]
        self.params = [-8.38357609e-03, 2.37367164e-02, -1.59672708e-04, 1.53623118e-06]

        # base_lla = [35.65492524, 128.39351431, 7] # KIAPI_Racing base
        # print("BASELLA!!!!!!!!!", self.base_lla)
        while 1: 
            if self.base_lla is not None:
                proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
                proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=self.base_lla[0], lon_0=self.base_lla[1], h_0=self.base_lla[2])
                self.transformer = Transformer.from_proj(proj_wgs84, proj_enu)
                break
            else:
                # print("baseLLA is NONE")
                pass

    def navatt_cb(self, msg):  # gain heading
        self.nav_hdg_last = self.nav_hdg
        self.real_nav_hdg = -(msg.heading*1e-5 - 90)%360
        if not self.hdg_hack:
            self.nav_hdg = self.real_nav_hdg 
            # self.headAcc = 0
        else:
            self.nav_hdg = 0
        self.nav_roll = msg.roll*1e-5
        self.nav_pitch = msg.pitch*1e-5

    def navpvt_cb(self, msg): # gain position
        if self.transformer is not None and (self.corr_can_velocity is not None):
            self.nav_pos_last = self.nav_pos
            lat = msg.lat*1e-7
            lon = msg.lon*1e-7
            x, y, _= self.transformer.transform(lon, lat, 7)
            self.real_nav_pos = [x, y]

            # self.headAcc = msg.headAcc
            if not self.pos_hack:
                self.nav_pos = self.real_nav_pos
                self.hAcc = msg.hAcc
            else:
                self.nav_pos = [0, 0]
                self.hAcc = 900
            
            if not self.hdg_hack:
                self.headAcc = msg.headAcc
                # self.headAcc = 0
            else:
                self.headAcc = 90000

            self.llh = [lat, lon]
            # change
            self.pvt_cb = True
            self.gspeed = msg.gSpeed
            if self.hAcc > 50 and self.corr_can_velocity * 3.6 > 10:
                self.pos_invalid_cnt += 1
            else:
                self.pos_invalid_cnt = 0

            if self.headAcc > 50000 and self.corr_can_velocity * 3.6 > 10:
                self.hdg_invalid_cnt += 1
            else:
                self.hdg_invalid_cnt = 0 
        
    def imu_cb(self, msg):
        # change
        self.imu_header_last = self.imu_header
        self.imu_angular_velocity_last = self.imu_angular_velocity
        self.imu_linear_acceleration_last = self.imu_linear_acceleration
        self.accel_last = self.accel
        self.gyro_last = self.gyro

        self.imu_header = msg.header
        self.imu_angular_velocity = msg.angular_velocity
        self.imu_linear_acceleration = msg.linear_acceleration

        self.accel = np.array([self.imu_linear_acceleration.x, self.imu_linear_acceleration.y, self.imu_linear_acceleration.z])
        self.gyro = np.array([self.imu_angular_velocity.x, self.imu_angular_velocity.y, self.imu_angular_velocity.z])

        if self.imu_header_last is not None:
            ds = self.imu_header.stamp.secs - self.imu_header_last.stamp.secs
            dns = self.imu_header.stamp.nsecs - self.imu_header_last.stamp.nsecs
            dt = ds + dns*1e-9
            if self.q is not None:
                self.q = self.madgwick.updateIMU(q=self.q, gyr=(self.gyro+self.gyro_last)/2, acc=(self.accel+self.accel_last)/2, dt=dt)
                imu_hdg_tmp = np.rad2deg(np.arctan2(2.0*(self.q[0]*self.q[3] + self.q[1]*self.q[2]), 1.0 - 2.0*(self.q[2]**2 + self.q[3]**2)))

                if self.curved:
                    constant_offset = 0
                else:
                    constant_offset = 0 

                self.imu_hdg = (imu_hdg_tmp + constant_offset)%360
    
    def fix_cb(self, msg): # gain header
        self.nav_header_last = self.nav_header
        self.nav_header = msg.header

    def system_status_cb(self, msg):
        self.base_lla = msg.baseLLA
    
    def canoutput_cb(self, msg): # gain velocity, steering angle
        self.canoutput_update()
        vRR = float(msg.WHEEL_SPD_RR.data)
        vRL = float(msg.WHEEL_SPD_RL.data) 
        self.can_velocity = (vRR + vRL)/7.2 # [m/s]
        self.corr_can_velocity = (self.can_velocity*3.6 \
                                  + self.params[0] + self.params[1]*(self.can_velocity*3.6) \
                                    + self.params[2]*((self.can_velocity*3.6)**2) \
                                        + self.params[3]*((self.can_velocity*3.6)**3))/3.6 # [m/s]
        if self.nav_pos_last[0] is not None:
            self.nav_velocity = (((self.nav_pos_last[0] - self.nav_pos[0])**2+(self.nav_pos_last[1] - self.nav_pos[1])**2)**0.5)*20
        # print(f" nav vel: {self.nav_velocity}\n can vel: {self.can_velocity_last}\ncorr vel: {self.corr_can_velocity_last}\n  error: {abs(self.corr_can_velocity_last - self.nav_velocity)}\n")
        # try:
        #     print(f"verror: {abs(self.corr_can_velocity_last - self.nav_velocity)}")
        # except:
        #     pass

        handle_ang = float(msg.StrAng.data)
        self.can_steer = handle_ang*self.steer_scale_factor 
    
    def canoutput_update(self):
        self.can_velocity_last = self.can_velocity
        self.corr_can_velocity_last = self.corr_can_velocity
        self.can_steer_last = self.can_steer

    def lanedata_cb(self, msg):
        # self.curr_lane_id = str(msg.currentLane.id.data)        
        self.laneNumber = msg.currentLane.laneNumber.data
        if msg.currentLane.id.data in self.curve_list:
            self.curved = True
        else:
            self.curved = False

    def publish(self, heading, enu):
        pos_msg = Pose()
        pos_msg.position.x = enu[0]
        pos_msg.position.y = enu[1]
        # pos_msg.position.z = not used
        pos_msg.orientation.x = self.llh[0]
        pos_msg.orientation.y = self.llh[1]
        pos_msg.orientation.z = heading
        # pos_msg.orientation.w = not used
        self.best_pose_pub.publish(pos_msg)
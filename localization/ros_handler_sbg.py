import rospy

from drive_msgs.msg import *
from sbg_driver.msg import SbgEkfEuler, SbgEkfNav, SbgImuData
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
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
        self.imu_ang_vel = None
        self.imu_lin_acc = None

        self.can_vel = None
        self.can_steer = None
        self.corr_can_vel = None

        self.can_vel_last = None
        self.can_steer_last = None
        self.corr_can_vel_last = None

        # tmp
        self.hdg_hack = False
        self.pos_hack = False
        self.nav_vel = 0

        # change
        self.curve_list = ['1', '7', '8', '9', '10', '11', '15', '16', '17', '21', '22', '23', '24', 
                           '25', '26', '27', '36', '37', '38', '39', '43', '44', '45', '46', '47', '54', 
                           '59', '60', '61', '62', '63', '68', '69', '70', '72', '73', '78', '79', '80']
        self.curved = False

        self.nav_cb = False
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
        rospy.Subscriber('/sbg/ekf_euler', SbgEkfEuler, self.sbgeuler_cb)
        rospy.Subscriber('/sbg/ekf_nav', SbgEkfNav, self.sbgnav_cb)
        rospy.Subscriber('/sbg/imu_data', SbgImuData, self.sbgimu_cb)
        rospy.Subscriber('/CANOutput', CANOutput, self.canoutput_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        rospy.Subscriber('/LaneData', LaneData, self.lanedata_cb)

        # tmp
        self.pub_hdg = rospy.Subscriber("/heading_hack", Bool, self.hdg_cb)
        self.pub_pos = rospy.Subscriber("/position_hack", Bool, self.pos_cb)

        self.best_pose_pub = rospy.Publisher('/best/pose', Pose, queue_size=1)
        ## Sensot health
        self.nav_health_pub = rospy.Publisher('/nav_health', Bool, queue_size=1)

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

    def sbgeuler_cb(self, msg):  # gain heading
        self.nav_hdg_last = self.nav_hdg
        self.real_nav_hdg = -(np.rad2deg(msg.angle.Yaw) - 90)%360
        if not self.hdg_hack:
            self.nav_hdg = self.real_nav_hdg 
        else:
            self.nav_hdg = 0
        self.nav_roll = np.rad2deg(msg.angle.Roll)
        self.nav_pitch = np.rad2deg(msg.angle.Pitch)
        self.headAcc = msg.accuracy

    def sbgnav_cb(self, msg): # gain position
        self.nav_pos_last = self.nav_pos
        lat = msg.latitude
        lon = msg.longitude
        x, y, _= self.transformer.transform(lon, lat, 7)
        self.real_nav_pos = [x, y]
        if not self.pos_hack:
            self.nav_pos = self.real_nav_pos
        else:
            self.nav_pos = [0, 0]

        self.llh = [lat, lon]
        # change
        self.nav_cb = True
        self.gspeed = np.sqrt(msg.velocity.x**2 + msg.velocity.y**2)
        self.hAcc = np.sqrt(msg.position_accuracy.x**2 + msg.position_accuracy.y**2)
    
    def sbgimu_cb(self, msg):
        # change
        self.imu_header_last = self.imu_header
        self.imu_ang_vel_last = self.imu_ang_vel
        self.imu_lin_acc_last = self.imu_lin_acc
        self.accel_last = self.accel
        self.gyro_last = self.gyro

        self.imu_header = msg.header
        self.imu_ang_vel = msg.gyro
        self.imu_lin_acc = msg.accel

        self.accel = np.array([self.imu_lin_acc.x, self.imu_lin_acc.y, self.imu_lin_acc.z])
        self.gyro = np.array([self.imu_ang_vel.x, self.imu_ang_vel.y, self.imu_ang_vel.z])

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
    
    def canoutput_cb(self, msg): # gain vel, steering angle
        self.canoutput_update()
        vRR = float(msg.WHEEL_SPD_RR.data)
        vRL = float(msg.WHEEL_SPD_RL.data) 
        self.can_vel = (vRR + vRL)/7.2 # [m/s]
        self.corr_can_vel = (self.can_vel*3.6 \
                                  + self.params[0] + self.params[1]*(self.can_vel*3.6) \
                                    + self.params[2]*((self.can_vel*3.6)**2) \
                                        + self.params[3]*((self.can_vel*3.6)**3))/3.6 # [m/s]
        if self.nav_pos_last[0] is not None:
            self.nav_vel = (((self.nav_pos_last[0] - self.nav_pos[0])**2+(self.nav_pos_last[1] - self.nav_pos[1])**2)**0.5)*20
        # print(f" nav vel: {self.nav_vel}\n can vel: {self.can_vel_last}\ncorr vel: {self.corr_can_vel_last}\n  error: {abs(self.corr_can_vel_last - self.nav_vel)}\n")
        # try:
        #     print(f"verror: {abs(self.corr_can_vel_last - self.nav_vel)}")
        # except:
        #     pass

        handle_ang = float(msg.StrAng.data)
        self.can_steer = handle_ang*self.steer_scale_factor 
    
    def canoutput_update(self):
        self.can_vel_last = self.can_vel
        self.corr_can_vel_last = self.corr_can_vel
        self.can_steer_last = self.can_steer

    def lanedata_cb(self, msg):
        # self.curr_lane_id = str(msg.currentLane.id.data)
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
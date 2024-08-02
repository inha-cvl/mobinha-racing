import rospy
import numpy as np

from drive_msgs.msg import *
from std_msgs.msg import Float32
from ublox_msgs.msg import NavATT, NavPVT
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Pose2D
from navatt_subs import NAVATT
from navpvt_subs import NAVPVT
from imu_meas_subs import IMUMEAS

from pyproj import Proj, Transformer

class ROSHandler():
    def __init__(self, map):
        rospy.init_node('localization', anonymous=False)
        self.MAP = map
        self.set_values()
        self.set_protocol()

    def set_values(self):
        self.heading_fixed = False
        self.navatt = NAVATT()
        self.navpvt = NAVPVT()
        self.imumeas = IMUMEAS()
        self.map_name = None
        self.transformer = None
        self.local_pose = [0,0]   
        self.curr_lane_id = None
        
        self.nav_header = None
        self.nav_pos = None
        self.nav_heading = None

        self.nav_header_last = None
        self.nav_pos_last = None
        self.nav_heading_last = None
        self.nav_velocity_last = None

        self.can_velocity = None
        self.can_steer = None
        self.corr_can_velocity = None

        self.can_velocity_last = None
        self.can_steer_last = None

        self.can_velocity_last = None
        self.can_steer_last = None
        self.corr_can_velocity_last = None

        self.wheelbase = 2.72
        self.steer_scale_factor = 36.2/500

        self.s_params = [-1.69519446e-01, 3.14832448e-02, -2.42469118e-04, 1.68413777e-06]
        self.c_params = [-3.04750083e-01, 4.43420297e-02, -6.07069742e-04, 4.46079605e-06]
        self.params = [-8.38357609e-03, 2.37367164e-02, -1.59672708e-04, 1.53623118e-06]

        base_lla = [35.65492524, 128.39351431, 7] # KIAPI_Racing base
        proj_wgs84 = Proj(proj='latlong', datum='WGS84') 
        proj_enu = Proj(proj='aeqd', datum='WGS84', lat_0=base_lla[0], lon_0=base_lla[1], h_0=base_lla[2])
        self.transformer = Transformer.from_proj(proj_wgs84, proj_enu)

    def set_protocol(self):
        rospy.Subscriber("/ublox/navatt", NavATT, self.navatt.callback)
        rospy.Subscriber("/ublox/navpvt", NavPVT, self.navpvt_cb)
        rospy.Subscriber("/ublox/fix", NavSatFix, self.fix_cb)
        rospy.Subscriber("/ublox/imu_meas", Imu, self.imumeas.callback)
        rospy.Subscriber('/CANOutput', CANOutput, self.canoutput_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        rospy.Subscriber('/LaneData', LaneData, self.lanedata_cb)
        
        # self.heading_pub = rospy.Publisher('/localization/heading', Float32, queue_size=1)
        # self.position_pub = rospy.Publisher('/localization/position', Pose, queue_size=1)
        # self.kf_heading_pub = rospy.Publisher('/kf/heading', Float32, queue_size=1)
        self.kf_pose_pub = rospy.Publisher('/kf/pose', Pose2D, queue_size=1)

    def navpvt_cb(self, msg): # gain position, heading
        self.navpvt_update()
        lat = msg.lat*1e-7
        lon = msg.lon*1e-7
        x, y, _= self.transformer.transform(lon, lat, 7)
        self.nav_pos = (x, y)
        self.nav_heading = -(msg.heading*1e-5 - 90)%360 
    
    def navpvt_update(self):
        self.nav_pos_last = self.nav_pos
        self.nav_heading_last = self.nav_heading
    
    def fix_cb(self, msg): # gain header
        self.fix_update()
        self.nav_header = msg.header

    def fix_update(self):
        self.nav_header_last = self.nav_header

    def system_status_cb(self, msg):
        if msg.headingSet.data == 1:
            self.heading_fixed = True 
        else:
            self.heading_fixed = False
    
    def canoutput_cb(self, msg): # gain velocity, steering angle
        self.canoutput_update()
        vRR = float(msg.WHEEL_SPD_RR.data)
        vRL = float(msg.WHEEL_SPD_RL.data) 
        self.can_velocity = (vRR + vRL)/7.2 # [m/s]
        self.corr_can_velocity = (self.can_velocity*3.6 \
                                  + self.params[0] + self.params[1]*(self.can_velocity*3.6) \
                                    + self.params[2]*((self.can_velocity*3.6)**2) \
                                        + self.params[3]*((self.can_velocity*3.6)**3))/3.6 # [m/s]

        handle_ang = float(msg.StrAng.data)
        self.can_steer = handle_ang*self.steer_scale_factor 
    
    def canoutput_update(self):
        self.can_velocity_last = self.can_velocity
        self.corr_can_velocity_last = self.corr_can_velocity
        self.can_steer_last = self.can_steer

    def lanedata_cb(self, msg):
        self.curr_lane_id = str(msg.currentLane.id.data)

    def publish(self, heading, position):
        pos_msg = Pose2D()
        pos_msg.x = position[0]
        pos_msg.y = position[1]
        pos_msg.theta = heading
        self.kf_pose_pub.publish(pos_msg)

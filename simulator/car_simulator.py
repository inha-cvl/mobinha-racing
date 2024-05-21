#!/usr/bin/python
import tf
import math
import pymap3d
import sys
import signal
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Vector3
from visualization_msgs.msg import Marker
from nmea_msgs.msg import Sentence
from std_msgs.msg import Int8
from libs.rviz_utils import *
from libs.vehicle import Vehicle

from drive_msgs.msg import *

def signal_handler(sig, frame):
    sys.exit(0)

def Sphere(ns, id_, data, scale, color):
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = color[0]/255
    marker.color.g = color[1]/255
    marker.color.b = color[2]/255
    marker.color.a = 0.9
    marker.pose.position.x = data[0]
    marker.pose.position.y = data[1]
    marker.pose.position.z = 0
    return marker

class CarSimulator:
    def __init__(self, map):
        wheelbase = 2.97

        if map == 'songdo-site':
            self.base_lla = [37.383378,126.656798,7] # Sondo-Site
            self.ego = Vehicle(-3800.520, 3840.930, -3.133, 0.0, wheelbase) #Songdo
        elif map=='songdo':
            self.base_lla = [37.3888319,126.6428739, 7.369]
            #self.ego = Vehicle(148.707, 310.741,2.478, 0.0, 2.65)
            self.ego = Vehicle(147.547, 311.788, 2.478, 0.0, wheelbase)  
        elif map == 'KIAPI':
            self.base_lla = [35.64588122580907,128.40214778762413, 46.746]
            self.ego = Vehicle(-127.595, 418.819, 2.380, 0.0, wheelbase)
        elif map == 'Pangyo':
            self.base_lla = [37.39991792889962, 127.11264200835348,7]
            self.ego = Vehicle(-10.687, 0.029, -3.079,0,wheelbase)
        elif map == 'Harbor':
            self.base_lla = [37.42390324724057, 126.60753475932731, 7]
            self.ego = Vehicle(559.144, -112.223, 3.074, 0, wheelbase)
        elif map == 'KIAPI_Racing':
            self.base_lla = [35.64750540757964, 128.40264207604886, 7]
            self.ego = Vehicle(0, 0, 1.664, 0, wheelbase)

        self.roll = 0.0
        self.pitch = 0.0

        self.ego_car = CarViz('ego_car', 'ego_car_info', [0, 0, 0], [241, 76, 152, 1])
        self.ego_car_info = CarInfoViz('ego_car', 'ego_car', '',[0,0,0])
        self.br = tf.TransformBroadcaster()

        self.mode = 0
        self.signal = 0
        self._steer = 0
        self._accel = 0
        self._brake = 0

        self.pub_viz_car = rospy.Publisher('/viz/car', Marker, queue_size=1)
        self.pub_viz_car_info = rospy.Publisher('/viz/car_info', Marker, queue_size=1)
        self.pub_nmea_sentence = rospy.Publisher('/nmea_sentence', Sentence, queue_size=1)
        self.pub_can_output = rospy.Publisher('/CANOutput', CANOutput, queue_size=1)

        rospy.Subscriber('/control/target_actuator', Actuator, self.target_actuator_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.init_pose_cb)

    def init_pose_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y,
                      orientation.z, orientation.w)
        self.roll, self.pitch, yaw = tf.transformations.euler_from_quaternion(
            quaternion)
        self.ego.set(x, y, yaw)

    def target_actuator_cb(self, msg):
        self._steer = msg.steer.data
        self._accel = msg.accel.data
        self._brake = msg.brake.data
   
    def system_status_cb(self, msg):
        self.mode = msg.systemMode.data
        self.signal = msg.systemSignal.data
     
    def run(self):
        rate = rospy.Rate(20)
        cnt = 0
        while not rospy.is_shutdown():
            dt = 0.05
            if self.mode == 1:
                x, y, yaw, v = self.ego.next_state(dt, self._steer, self._accel, self._brake)
            else:
                x, y, yaw, v = self.ego.x, self.ego.y, self.ego.yaw, self.ego.v
            self.yaw = math.degrees(yaw)
            lat, lon, alt = pymap3d.enu2geodetic(x, y, 0, self.base_lla[0], self.base_lla[1], self.base_lla[2])
            nmea_sentence = Sentence()
            if cnt % 2 == 0:
                sentence = f"$GPGGA,123519,{lat*100},N,{lon*100},E,1,08,0.9,545.4,M,46.9,M,,*47"
            else:
                sentence = f"$GPHDT,{self.yaw},T*hh"
            nmea_sentence.header.stamp = rospy.Time.now()
            nmea_sentence.header.frame_id = "gps"
            nmea_sentence.sentence = sentence
            self.pub_nmea_sentence.publish(nmea_sentence)

            can_output = CANOutput()
            mode_to_signal = {
                0:('Ready', 'Ready'),# Off 
                1:('All_On', 'All_On'), # On
                2:('EPS_On', 'EPS_On'), # Steering Only
                3:('ACC_On', 'ACC_On') # Acce/Brake Only
            }
            (eps_status, acc_status) = mode_to_signal.get(self.mode)
            can_output.EPS_Control_Status.data = eps_status
            can_output.ACC_Control_Board_Status.data = acc_status
            can_output.WHEEL_SPD_RR.data = str((7.2/2)*v)
            can_output.WHEEL_SPD_RL.data = str((7.2/2)*v)
            signal_to_turn = {
                0:['Off','Off','Off'],
                1:['On','Off','Off'],
                2:['Off','Off','On'],
                3:['Off','On','Off'],
                4:['Off', 'Off', 'Off'],
            }
            turn_sig = signal_to_turn.get(self.signal)
            can_output.Turn_Left_En.data = turn_sig[0]
            can_output.Turn_Right_En.data = turn_sig[2]
            can_output.Hazard_En.data = turn_sig[1]
            can_output.G_SEL_DISP.data = 'D'
            can_output.Long_ACCEL.data = str(self._accel)
            can_output.BRK_CYLINDER.data = str(self._brake)
            can_output.StrAng.data = str(self._steer)

            self.pub_can_output.publish(can_output)

            info = f"{(v*3.6):.2f}km/h {self.yaw:.2f}deg"
            self.ego_car_info.text = info

            quaternion = tf.transformations.quaternion_from_euler(math.radians(self.roll), math.radians(self.pitch), math.radians(self.yaw))  # RPY
            self.br.sendTransform(
                (x, y, 0),
                (quaternion[0], quaternion[1],
                    quaternion[2], quaternion[3]),
                rospy.Time.now(),
                'ego_car',
                'world'
            )
            self.pub_viz_car.publish(self.ego_car)
            self.pub_viz_car_info.publish(self.ego_car_info)
            cnt += 1
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('car_simulator', anonymous=False)
    map = sys.argv[1]
    st = CarSimulator(map)
    st.run()

if __name__ == "__main__":
    main()
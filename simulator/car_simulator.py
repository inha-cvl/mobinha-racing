#!/usr/bin/python
import tf
import math
import pymap3d
import sys
import signal
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D
from libs.vehicle import Vehicle

from drive_msgs.msg import *

def signal_handler(sig, frame):
    sys.exit(0)

class CarSimulator:
    def __init__(self):
        self.ego = None
        self.base_lla = [0,0,0]
        self.mode = 0
        self.signal = 0
        self._steer = 0
        self._accel = 0
        self._brake = 0

        self.pub_simulator_pose = rospy.Publisher('/simulator/pose', Pose2D, queue_size=1)
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
        map_name = msg.mapName.data
        if self.ego == None:
            self.set_ego(map_name)
            self.base_lla = msg.baseLLA
        self.mode = msg.systemMode.data
        self.signal = msg.systemSignal.data
        self.kiapi_signal = msg.kiapiSignal.data
    
    def set_ego(self, map):
        wheelbase = 2.97
        if map == 'songdo-site':
            self.ego = Vehicle(-3800.520, 3840.930, -3.133, 0.0, wheelbase) #Songdo
        elif map=='songdo':
            self.ego = Vehicle(147.547, 311.788, 2.478, 0.0, wheelbase)  
        elif map == 'KIAPI':
            self.ego = Vehicle(-127.595, 418.819, 2.380, 0.0, wheelbase)
        elif map == 'Pangyo':
            self.ego = Vehicle(-10.687, 0.029, -3.079,0,wheelbase)
        elif map == 'Harbor':
            self.ego = Vehicle(559.144, -112.223, 3.074, 0, wheelbase)
        elif map == 'KIAPI_Racing' or map == 'KIAPI_Racing_Fast':
            #self.ego = Vehicle(10.759, 2.147, -2.99, 0, wheelbase) #3 lane first
            #self.ego = Vehicle(6.593, -6.417,-2.969, 0, wheelbase ) #1 lane firstv
            self.ego = Vehicle(66.799, 81.714,-0.681, 0, wheelbase ) #2 lane
            
        elif map == 'KCity':
            self.ego = Vehicle(167.259, 55.401, 2.810, 0, wheelbase)
        elif map == 'Solchan':
            self.ego = Vehicle(-0.881, -1.105, 0.829, 0, wheelbase)

    def run(self):
        rate = rospy.Rate(20)
        cnt = 0
        while not rospy.is_shutdown():
            if self.ego == None:
                continue
            dt = 0.05
            if self.mode == 1:
                x, y, yaw, v = self.ego.next_state(dt, self._steer, self._accel, self._brake)
            else:
                x, y, yaw, v = self.ego.x, self.ego.y, self.ego.yaw, self.ego.v
            self.yaw = math.degrees(yaw)
            lat, lon, alt = pymap3d.enu2geodetic(x, y, 0, self.base_lla[0], self.base_lla[1], self.base_lla[2])
            pose2d = Pose2D()
            pose2d.x = lat
            pose2d.y = lon
            pose2d.theta = self.yaw
            self.pub_simulator_pose.publish(pose2d)

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
                5:['Off', 'Off', 'Off']
            }
            turn_sig = signal_to_turn.get(self.signal)
            can_output.Turn_Left_En.data = turn_sig[0]
            can_output.Turn_Right_En.data = turn_sig[2]
            can_output.Hazard_En.data = turn_sig[1]
            can_output.G_SEL_DISP.data = 'D'
            can_output.LAT_ACCEL.data = str(self._accel)
            can_output.Long_ACCEL.data = str(self._accel)
            can_output.BRK_CYLINDER.data = str(self._brake)
            can_output.StrAng.data = str(self._steer)
            if self.kiapi_signal == 1:
                can_output.SIG_GO.data = 'On'
            elif self.kiapi_signal == 2:
                can_output.SIG_STOP.data = 'On'
            elif self.kiapi_signal == 3:
                can_output.SIG_SLOW_ON.data = 'On'
            elif self.kiapi_signal == 4:
                can_output.SIG_SLOW_OFF.data = 'On'
            elif self.kiapi_signal == 5:
                can_output.SIG_PIT_STOP.data = 'On'

            self.pub_can_output.publish(can_output)

            cnt += 1
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('car_simulator', anonymous=False)
    st = CarSimulator()
    st.run()

if __name__ == "__main__":
    main()
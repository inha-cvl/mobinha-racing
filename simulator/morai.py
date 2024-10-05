#!/usr/bin/python
import math
import tf
import rospy
import pymap3d

from drive_msgs.msg import *

from geometry_msgs.msg import Pose2D
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
from sensor_msgs.msg import Imu
from morai_msgs.msg import ObjectStatusList, CtrlCmd
from geometry_msgs.msg import Pose, PoseArray

class Morai:
    def __init__(self):
        #self.base_lla = [35.64750540757964, 128.40264207604886, 7]
        self.base_lla = [37.2292221592864,126.76912499027308, 7]
        self.pose = Pose2D()
        self.ctrl_msg = CtrlCmd()
        self.mode = 0
        self.egoxy = [0, 0, 0, 0]
        
        self.pub_sim_pose = rospy.Publisher('/simulator/pose', Pose2D, queue_size=1)
        self.pub_sim_object = rospy.Publisher('/simulator/objects', PoseArray, queue_size=1)
        self.pub_can_output = rospy.Publisher('/CANOutput', CANOutput, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)  # Vehicl Control
        rospy.Subscriber("/gps", GPSMessage, self.gps_cb)
        rospy.Subscriber("/imu", Imu, self.imu_cb)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_topic_cb)
        rospy.Subscriber("/Object_topic", ObjectStatusList,self.object_topic_cb)
        rospy.Subscriber('/control/target_actuator', Actuator, self.target_actuator_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        
    def gps_cb(self, msg):
        self.pose.x = msg.latitude
        self.pose.y = msg.longitude
    
    def imu_cb(self, msg):
        quaternion = (msg.orientation.x, msg.orientation.y,msg.orientation.z, msg.orientation.w)
        _, _,yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.pose.theta = math.degrees(yaw)
    
    def system_status_cb(self, msg):
        self.mode = msg.systemMode.data

    def ego_topic_cb(self, msg):
        self.egoxy[0] = msg.position.x
        self.egoxy[1] = msg.position.y

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
        
        if msg.velocity.x > 0:
            v = msg.velocity.x 
        else:
            v = 0
        can_output.WHEEL_SPD_RR.data = str((7.2/2)*v) 
        can_output.WHEEL_SPD_RL.data = str((7.2/2)*v)
        signal_to_turn = {
            0:['Off','Off','Off'],
            1:['On','Off','Off'],
            2:['Off','Off','On'],
            3:['Off','On','Off'],
            4:['Off', 'Off', 'Off'],
        }
        turn_sig = signal_to_turn.get(0)
        can_output.Turn_Left_En.data = turn_sig[0]
        can_output.Turn_Right_En.data = turn_sig[2]
        can_output.Hazard_En.data = turn_sig[1]
        can_output.G_SEL_DISP.data = 'D'
        can_output.Long_ACCEL.data = str(msg.accel)
        can_output.BRK_CYLINDER.data = str(msg.brake)
        can_output.StrAng.data = str(msg.wheel_angle)
        x, y, _ = pymap3d.geodetic2enu(self.pose.x, self.pose.y, 0, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        self.egoxy[2] = x
        self.egoxy[3] = y
        self.pub_can_output.publish(can_output)

    def target_actuator_cb(self, data):
        self.ctrl_msg.steering = math.radians(data.steer.data)
        self.ctrl_msg.accel = (data.accel.data/2)
        self.ctrl_msg.brake = (data.brake.data/3)

    def object_topic_cb(self, data):
        pose_array = PoseArray()
        dx = self.egoxy[2]-self.egoxy[0]
        dy = self.egoxy[3]-self.egoxy[1]
        for i, obj in enumerate(data.npc_list):
            pose = Pose()
            pose.position.x = obj.position.x + dx
            pose.position.y = obj.position.y + dy
            pose.position.z = 1
            pose.orientation.x = obj.velocity.x/3.6 if obj.velocity.x > 0 else 0
            pose.orientation.y = obj.heading
            pose_array.poses.append(pose)
        for i, obj in enumerate(data.obstacle_list):
            pose = Pose()
            pose.position.x = obj.position.x + dx
            pose.position.y = obj.position.y + dy
            pose.position.z = 1
            pose.orientation.x = obj.velocity.x/3.6 if obj.velocity.x > 0 else 0
            pose.orientation.y = obj.heading
            pose_array.poses.append(pose)
        self.pub_sim_object.publish(pose_array)

    def publisher(self):
        self.pub_sim_pose.publish(self.pose)
        self.ctrl_pub.publish(self.ctrl_msg)
        
    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publisher()
            rate.sleep()
    
def main():
    rospy.init_node('Morai', anonymous=False)
    m = Morai()
    m.run()
    
if __name__ == "__main__":
    main()
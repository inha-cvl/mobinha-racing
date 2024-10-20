import rospy
import sys
import signal
import tf
import math
import time

from drive_msgs.msg import *
from libs.vehicle import Vehicle
from geometry_msgs.msg import PoseStamped, Pose, PoseArray

def signal_handler(sig, frame):
    sys.exit(0)


class ObjectSimulator:
    def __init__(self):
        self.objects = []  # 객체들을 저장할 리스트
        self._steer = 0
        self._accel = 0
        self._brake = 0
        self.pub_sim_object = rospy.Publisher('/simulator/objects', PoseArray, queue_size=1)

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        rospy.Subscriber('/control/target_actuator', Actuator, self.target_actuator_cb)
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
    
    def vehicle_state_cb(self, msg: VehicleState):
        self.heading = msg.heading.data

    def goal_cb(self, msg):
        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        new_object = Vehicle(msg.pose.position.x, msg.pose.position.y, math.radians(self.heading), 30/3.6, 2.97)
        self.objects.append(new_object)  # 새로운 객체를 리스트에 추가

    def target_actuator_cb(self, msg):
        self._steer = 0  # msg.steer.data
        self._accel = msg.accel.data
        self._brake = msg.brake.data

    def publish_object(self):
        rate = rospy.Rate(20)
        dt = 0.05
        pose_array = PoseArray()
        while not rospy.is_shutdown():
            if self.objects:
                for obj in self.objects:
                    #x, y, yaw, v = obj.next_state(dt, self._steer, self._accel, 0)
                    #yaw = math.degrees(yaw)
                    
                    pose = Pose()
                    pose.position.x = obj.x
                    pose.position.y = obj.y
                    pose.position.z = 1
                    pose.orientation.x = 0
                    pose.orientation.y = obj.yaw
                    pose_array.poses.append(pose)
            self.pub_sim_object.publish(pose_array)
            pose_array = PoseArray()
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('object_simulator', anonymous=False)
    object_simulator = ObjectSimulator()
    object_simulator.publish_object()

if __name__ == "__main__":
    main()

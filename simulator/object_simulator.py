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
        
        self.object = None
        self.object_ready = False
        self._steer = 0
        self._accel = 0
        self._brake = 0
        self.pub_sim_object = rospy.Publisher('/simulator/objects', PoseArray, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        rospy.Subscriber('/control/target_actuator', Actuator, self.target_actuator_cb)
        

    def goal_cb(self, msg):
       quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
       _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
       self.object = Vehicle(msg.pose.position.x, msg.pose.position.y, yaw, 0, 2.97)
       self.object_ready = True
       
    def target_actuator_cb(self, msg):
        self._steer = msg.steer.data
        self._accel = msg.accel.data
        self._brake = msg.brake.data

    def publish_object(self):
        rate = rospy.Rate(20)
        start_time = time.time()
        vel = 0
        dt = 0.05
        while not rospy.is_shutdown():
            if self.object_ready:
                if time.time()-start_time > 3:
                    self.object_ready = False
                    vel = 0
                x,y,yaw,v = self.object.next_state(dt, self.object.yaw, self._accel, self._brake)
                v =+ vel
                yaw = math.degrees(yaw)
                pose_array = PoseArray()
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = 1
                pose.orientation.x = v 
                pose.orientation.y = yaw
                pose_array.poses.append(pose)
                vel += 0.01
            else:
                pose_array = PoseArray()
                start_time = time.time()
            self.pub_sim_object.publish(pose_array)

            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('object_simulator', anonymous=False)
    object_simulator = ObjectSimulator()
    object_simulator.publish_object()

if __name__ == "__main__":
    main()
#!/usr/bin/python
import math
import tf
import rospy
import numpy as np
import pymap3d

from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from morai_msgs.msg import GPSMessage, EgoVehicleStatus
from sensor_msgs.msg import Imu
from morai_msgs.msg import ObjectStatusList, CtrlCmd, Lamps
from geometry_msgs.msg import Pose, PoseArray, Vector3
from std_msgs.msg import Int8, Float32MultiArray
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox

from libs.rviz_utils import *

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


class Morai:
    def __init__(self):
        self.base_lla = [35.64750540757964, 128.40264207604886, 7]
        self.pose = Pose()
        self.ctrl_msg = CtrlCmd()
        self.lamps = Lamps()
        self.mode = 0
        self.egoxy = [0,0, 0, 0]
        
        self.ego_car = CarViz('ego_car', 'ego_car_info', [0, 0, 0], [241, 76, 152, 1])
        self.ego_car_info = CarInfoViz('ego_car', 'ego_car', '',[0,0,0])
        self.br = tf.TransformBroadcaster()

        self.pub_ego_car = rospy.Publisher('/car/ego_car', Marker, queue_size=1)
        self.pub_ego_car_info = rospy.Publisher('/car/ego_car_info', Marker, queue_size=1)
        self.pub_pose = rospy.Publisher('/car/pose', Pose, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd_0', CtrlCmd, queue_size=1)  # Vehicl Control
        self.track_box_pub = rospy.Publisher('/simulator/track_box', BoundingBoxArray, queue_size=1)
        self.mode_pub = rospy.Publisher('/car/mode', Int8, queue_size=1)
        self.pub_fake_obstacles = rospy.Publisher('/simulator/obstacle', MarkerArray, queue_size=1)
        rospy.Subscriber("/gps", GPSMessage, self.gps_cb)
        rospy.Subscriber("/imu", Imu, self.imu_cb)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_topic_cb)
        rospy.Subscriber("/Object_topic", ObjectStatusList,self.object_topic_cb)
        rospy.Subscriber('/selfdrive/actuator', Vector3, self.actuator_cb)
        rospy.Subscriber('/mode', Int8, self.mode_cb)
        
    def gps_cb(self, msg):
        self.pose.position.x = msg.latitude
        self.pose.position.y = msg.longitude
    
    def mode_cb(self, msg):
        self.mode = msg.data

    def imu_cb(self, msg):
        quaternion = (msg.orientation.x, msg.orientation.y,msg.orientation.z, msg.orientation.w)
        _, _,yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.pose.position.z = math.degrees(yaw)


    def ego_topic_cb(self, msg):
        self.pose.orientation.x = msg.velocity.x
        self.pose.orientation.y = msg.wheel_angle
        self.pose.orientation.z = msg.accel
        self.pose.orientation.w = msg.brake

        self.egoxy[0] = msg.position.x
        self.egoxy[1] = msg.position.y
     
        x, y, _ = pymap3d.geodetic2enu(self.pose.position.x, self.pose.position.y, 0, self.base_lla[0], self.base_lla[1], self.base_lla[2])
        h = self.pose.position.z
        v = msg.velocity.x
        self.ego_car_info.text = f"{(v*3.6):.2f}km/h {h:.2f}deg"
        qt = tf.transformations.quaternion_from_euler(0, 0, math.radians(h))  # RPY
        self.br.sendTransform((x,y, 0),(qt[0], qt[1],qt[2], qt[3]),rospy.Time.now(),'ego_car','world')
        self.egoxy[2] = x
        self.egoxy[3] = y

    def actuator_cb(self, data):
        self.ctrl_msg.steering = math.radians(data.x)
        self.ctrl_msg.accel = (data.y/100)*4
        self.ctrl_msg.brake = (data.z/100)*4

    def object_topic_cb(self, data):
        object_list = BoundingBoxArray()
        marker_array = MarkerArray()
        dx = self.egoxy[2]-self.egoxy[0]
        dy = self.egoxy[3]-self.egoxy[1]
        for i, obj in enumerate(data.npc_list):
            bbox = BoundingBox()
            pose = Pose()
            pose.position.x = obj.position.x + dx
            pose.position.y = obj.position.y + dy
            pose.orientation.z = obj.heading+90
            bbox.pose = pose
            bbox.value = obj.velocity.x/3.6
            bbox.label = 1
            object_list.boxes.append(bbox)
            marker_array.markers.append(Sphere(f'obj{i}', i, [pose.position.x,pose.position.y] , 5.0, (0,0,255)))
        for i, obj in enumerate(data.obstacle_list):
            bbox = BoundingBox()
            pose = Pose()
            pose.position.x = obj.position.x + dx
            pose.position.y = obj.position.y + dy
            pose.orientation.z = obj.heading+90
            bbox.pose = pose
            bbox.value = obj.velocity.x/3.6
            bbox.label = 1
            object_list.boxes.append(bbox)
            marker_array.markers.append(Sphere(f'obj{i}', i, [pose.position.x,pose.position.y] , 5.0, (0,0,255)))

        self.pub_fake_obstacles.publish(marker_array)
        self.track_box_pub.publish(object_list)

    def publisher(self):
        self.pub_pose.publish(self.pose)
        self.ctrl_pub.publish(self.ctrl_msg)
        self.mode_pub.publish(Int8(self.mode))
        self.pub_ego_car.publish(self.ego_car)
        self.pub_ego_car_info.publish(self.ego_car_info)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publisher()
            rate.sleep()
    
def main():
    rospy.init_node('Morai', anonymous=False)
    m = Morai()
    m.run()
    
if __name__ == "__main__":
    main()
import rospy

from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray, Marker
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped, Pose, PoseArray

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Float32MultiArray
from drive_msgs.msg import *
import math
import perception_handler as ph

import tf


class ROSHandler():
    def __init__(self):
        rospy.init_node('perception', anonymous=False)
        self.set_values()
        self.set_protocol()
    
    def set_values(self):
        self.radar_objects = []
        self.est_veh_spd = 0
        self.img = None
        self.bounding_boxes = []
        self.bridge = CvBridge()


    def set_protocol(self):
        rospy.Subscriber('/RadarObjectArray', RadarObjectArray, self.radar_object_array_cb)
        rospy.Subscriber('/ADAS_DRV',Float32MultiArray, self.adas_drv_cb )
        rospy.Subscriber('/camera/image_color', Image, self.image_cb)
        rospy.Subscriber('/detection_results', Detection2DArray, self.detection_cb)
        
        self.radar_objects_marker_pub = rospy.Publisher('/RadarObjects', MarkerArray, queue_size=1)
        self.processed_image_pub = rospy.Publisher('/camera/processed_image', Image, queue_size=10)
        self.fusion_objects_pub = rospy.Publisher('/perception/fusion_objects', PoseArray, queue_size=1)

    def adas_drv_cb(self, msg):
        self.est_veh_spd = msg.data[5]

    def radar_object_array_cb(self, msg):
        radar_objects = []
        for ro in msg.radarObjects:
            if ro.qualLvl.data > 33  and ro.alvAge.data > 10:
                heading, velocity = ph.calculate_radar_heading_velocity(ro.relPosX.data+2.325, ro.relPosY.data, ro.relVelX.data, ro.relVelY.data)
                if velocity == 0:
                    obj_vel = 0
                else:
                    obj_vel = velocity + self.est_veh_spd
                # obj = [ro.relPosX.data+2.325, ro.relPosY.data, heading, obj_vel, ro.alvAge, ro.qualLvl.data]
                obj = [ro.relPosX.data+2.35, ro.relPosY.data, heading, obj_vel, int(ro.alvAge.data), int(ro.qualLvl.data)]
                radar_objects.append(obj)
        self.radar_objects = radar_objects

    def image_cb(self, msg):
        bridge = CvBridge()
        self.img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def detection_cb(self, detection_array):
        self.bounding_boxes = []
        for detection in detection_array.detections:
            bbox = detection.bbox
            xmin = int(bbox.center.x - bbox.size_x / 2)
            ymin = int(bbox.center.y - bbox.size_y / 2)
            xmax = int(bbox.center.x + bbox.size_x / 2)
            ymax = int(bbox.center.y + bbox.size_y / 2)
            self.bounding_boxes.append((xmin, ymin, xmax, ymax))



    def publish(self, positions):
        
        marker_array = MarkerArray()

        for i, (x, y, o1, o2, o3, o4) in enumerate(positions):

            marker = Marker()
            marker.header.frame_id = "hesai_lidar"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.05)
            marker.pose.position.x = x+0.8
            marker.pose.position.y = y
            marker.pose.position.z = -1.2  # Assume ground level for simplicity
            quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(o1))
            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]
            marker.scale.x = 1.6
            marker.scale.y = 1.6
            marker.scale.z = 1.6
            marker.color.a = 0.5  # Don't forget to set the alpha!
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)

            marker = Marker()
            marker.header.frame_id = "hesai_lidar"
            marker.ns = "objects"
            marker.id = i+32
            marker.type = Marker.TEXT_VIEW_FACING
            marker.lifetime = rospy.Duration(0.05)
            marker.scale.z = 1
            marker.color.r = 1
            marker.color.g = 1
            marker.color.b = 1
            marker.color.a = 1.0
            marker.pose.position.x = x+0.8
            marker.pose.position.y = y
            marker.pose.position.z = -1.2+0.65
            marker.text = f"{(x-2.325):.1f}m {o2:.1f}m/s"
            #°
            marker_array.markers.append(marker)

        self.radar_objects_marker_pub.publish(marker_array)

    def publish_result_img(self, frame):
        try:
            # Convert OpenCV image to ROS Image message
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            # Publish the Image message
            self.processed_image_pub.publish(image_message)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert and publish image: {e}")

    def publish_object_array(self, list):
        pose_array = PoseArray()
        for obj in list:
            pose = Pose()
            pose.position.x = obj[0]+2.35
            pose.position.y = obj[1]
            pose.position.z = 1
            pose.orientation.x = obj[3]
            pose.orientation.y = obj[2]
            pose_array.poses.append(pose)
        self.fusion_objects_pub.publish(pose_array)


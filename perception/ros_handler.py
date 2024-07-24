import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import Image, CompressedImage

class ROSHandler():
    def __init__(self):
        rospy.init_node('perception', anonymous=False)
        self.set_values()
        self.set_protocol()
    
    def set_values(self):
        self.local_pose = []
        self.local_path = []
        

    def set_protocol(self):
        rospy.Subscriber('/camera/image_raw', Image, self.front_center_camera_cb)
        self.processed_image_pub = rospy.Publisher('/camera/processed_image', Image, queue_size=10)
        self.box_detection_pub = rospy.Publisher('/detection_markers', MarkerArray, queue_size=1)

    def front_center_camera_cb(self, msg):    
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = frame.shape
        y_start = int(2.05 * height / 5)
        y_end = int(3.78 * height / 5)
        
        # remove background
        # fgmask = self.fgbg.apply(frame)
        # self.frame = fgmask[y_start:y_end, :]

        self.frame = frame[y_start:y_end, :]


    def publish(self, frame, positions):
        try:
            # Convert OpenCV image to ROS Image message
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            # Publish the Image message
            self.processed_image_pub.publish(image_message)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to convert and publish image: {e}")
        
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(positions):
            marker = Marker()
            marker.header.frame_id = "camera_frame"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0  # Assume ground level for simplicity
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0  # Don't forget to set the alpha!
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)
        self.box_detection_pub.publish(marker_array)

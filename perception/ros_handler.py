import rospy
from visualization_msgs.msg import MarkerArray, Marker
from drive_msgs.msg import *

class ROSHandler():
    def __init__(self):
        rospy.init_node('perception', anonymous=False)
        self.set_values()
        self.set_protocol()
    
    def set_values(self):
        self.radar_objects = []

    def set_protocol(self):
        rospy.Subscriber('/RadarObjectArray', RadarObjectArray, self.radar_object_array_cb)
        self.radar_objects_marker_pub = rospy.Publisher('/RadarObjects', MarkerArray, queue_size=1)
    
    def radar_object_array_cb(self, msg):
        radar_objects = []
        for ro in msg.radarObjects:
            obj = [ro.relPosX.data, ro.relPosY.data]
            radar_objects.append(obj)
        self.radar_objects = radar_objects

    def publish(self, positions):
        
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(positions):
            marker = Marker()
            marker.header.frame_id = "radar_frame"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.05)
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
        self.radar_objects_marker_pub.publish(marker_array)

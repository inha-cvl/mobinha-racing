#!/usr/bin/env python
import rospy
from visualization_msgs.msg import MarkerArray

def callback(msg):
    # frame_id를 변경
    for marker in msg.markers:
        marker.header.frame_id = "world"  # 원하는 frame_id로 변경

    # 수정된 메시지를 다시 퍼블리시
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('marker_array_frame_changer')

    # MarkerArray 메시지를 구독
    rospy.Subscriber("/RadarObjects", MarkerArray, callback)

    # 수정된 메시지를 퍼블리시할 새로운 토픽
    pub = rospy.Publisher("/radar_objects", MarkerArray, queue_size=10)

    rospy.spin()

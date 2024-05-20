#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseStamped

def callback(data):
    # 새 PoseStamped 메시지 생성
    pose_stamped = PoseStamped()

    # 현재 시간을 timestamp로 설정
    pose_stamped.header.stamp = rospy.Time.now()
    
    # frame_id 설정 (필요에 따라 변경)
    pose_stamped.header.frame_id = "map"

    # Pose 데이터 복사
    pose_stamped.pose = data

    # 변환된 PoseStamped 메시지를 발행
    pub.publish(pose_stamped)
    rospy.loginfo("Published PoseStamped message with timestamp")

# 노드 초기화
rospy.init_node('pose_to_posestamped')

# Pose 메시지를 구독
sub = rospy.Subscriber('/car/hlv_pose', Pose, callback)

# PoseStamped 메시지를 발행할 퍼블리셔 생성
pub = rospy.Publisher('output_posestamped', PoseStamped, queue_size=10)

# ROS 메인 루프 시작
rospy.spin()

#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8

def publish_nav_health():
    # ROS 노드 초기화
    rospy.init_node('nav_health_publisher', anonymous=True)
    
    # /nav_health 토픽에 Int8 메시지를 publish할 publisher 객체 생성
    pub = rospy.Publisher('/nav_health', Int8, queue_size=10)
    
    # 5Hz로 publish할 속도를 설정 (1초에 5번)
    rate = rospy.Rate(5) # 5 Hz

    while not rospy.is_shutdown():
        try:
            # 사용자로부터 입력 받기
            user_input = input("Enter a value (Int8) to publish to /nav_health: ")

            # 입력 값을 Int8 타입으로 변환 후 메시지로 설정
            nav_health_value = Int8()
            nav_health_value.data = int(user_input)

            # 메시지를 /nav_health 토픽에 publish
            pub.publish(nav_health_value)
            rospy.loginfo("Published: %d", nav_health_value.data)

            # 5Hz로 publish 반복
            rate.sleep()
        
        except ValueError:
            rospy.logwarn("Please enter a valid integer.")

        except KeyboardInterrupt:
            rospy.loginfo("Shutting down publisher.")
            break

if __name__ == '__main__':
    try:
        publish_nav_health()
    except rospy.ROSInterruptException:
        pass

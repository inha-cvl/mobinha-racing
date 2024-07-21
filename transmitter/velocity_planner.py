#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def publish_target_v():
    rospy.init_node('target_v_publisher', anonymous=True)
    pub = rospy.Publisher('/target_v_tmp', Float32, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    target_v = 0.0
    while not rospy.is_shutdown():
        try:
            user_input = input("Enter a float value for /target_v (or 'exit' to quit): ")
            if user_input.lower() == 'exit':
                break
            target_v = float(user_input)
        except ValueError:
            rospy.logerr("Invalid input. Please enter a valid float value.")
            continue

        rospy.loginfo(f"Publishing: {target_v}")
        pub.publish(target_v)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_target_v()
    except rospy.ROSInterruptException:
        pass

import matplotlib.pyplot as plt
import rospy
from navpvt_subs import NAVPVT
from navatt_subs import NAVATT
from std_msgs.msg import Float32, Bool
import numpy as np
from ublox_msgs.msg import NavPVT, NavATT

navatt_data = []
hacked_data = []
compensated_data = []

true_heading = 0
def trueHeading_callback(msg):
    global true_heading
    true_heading = msg.data

def update_plot(data_len):
    for el in [compensated_data, navatt_data, hacked_data]:
        if len(el) > data_len:
            el.pop(0)

    ax.clear()

    ax.plot(compensated_data, label='Compensated Heading')
    ax.plot(navatt_data, linestyle="--", label='True Heading')
    ax.plot(hacked_data, label='Hacked Heading')
    ax.legend()
    plt.draw()
    plt.grid()
    plt.ylim(-5, 365)
    plt.pause(0.001)

if __name__ == "__main__":
    rospy.init_node("heading_compare_node")
    navpvt = NAVPVT()
    navatt = NAVATT()
    rospy.Subscriber("/ublox/navpvt", NavPVT, navpvt.callback_for_imuHeadingTests)
    rospy.Subscriber("/ublox/navatt", NavATT, navatt.callback)
    rospy.Subscriber("/localization/heading", Float32, trueHeading_callback)
    rospy.Subscriber("/heading_hack_flag", Bool, navpvt.hacked_cb)

    fig, ax = plt.subplots()

    while not rospy.is_shutdown():
        compensated_data.append(true_heading)
        navatt_data.append(navatt.heading)
        hacked_data.append(navpvt.heading)
        print(true_heading, navatt.heading, navpvt.heading)
        update_plot(100)
        #rospy.sleep(0.05)



import rospy
import tf


from drive_msgs.msg import *
from visualization_msgs.msg import Marker

from libs.rviz_utils import *

class MarkerPublisher:
    def __init__(self):
        rospy.init_node('marker_publisher', anonymous=False)

        self.ego_car = CarViz('ego_car', 'ego_car_info', [0, 0, 0], [241, 76, 152, 1])
        self.ego_car_info = CarInfoViz('ego_car', 'ego_car', '',[0,0,0])
        self.br = tf.TransformBroadcaster()
        self.x, self.y = 0,0

        self.pub_viz_car = rospy.Publisher('/viz/car', Marker, queue_size=1)
        self.pub_viz_car_info = rospy.Publisher('/viz/car_info', Marker, queue_size=1)
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/NavigationData', NavigationData, self.navigation_data_cb)

        rospy.spin()

    def vehicle_state_cb(self, msg):
        yaw = msg.heading.data
        v = msg.velocity.data

        info = f"{(v*3.6):.2f}km/h {yaw:.2f}deg"
        self.ego_car_info.text = info

        #print(self.x, self.y, yaw)
        quaternion = tf.transformations.quaternion_from_euler(math.radians(0), math.radians(0), math.radians(yaw))  # RPY
        self.br.sendTransform(
            (self.x, self.y, 0),
            (quaternion[0], quaternion[1],quaternion[2], quaternion[3]),
            rospy.Time.now(),
            'ego_car',
            'world'
        )
        self.pub_viz_car.publish(self.ego_car)
        self.pub_viz_car_info.publish(self.ego_car_info)

    
    def navigation_data_cb(self, msg):
        self.x = msg.currentLocation.x
        self.y = msg.currentLocation.y
    
if __name__ == "__main__":
    marker_publisher = MarkerPublisher()


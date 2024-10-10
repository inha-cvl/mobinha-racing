import rospy

from drive_msgs.msg import *
from libs.point import Point 

from visualization_msgs.msg import Marker

class ROSHandler():
    def __init__(self):
        rospy.init_node('control', anonymous=False)
        
        self.set_values()
        self.set_publisher_protocol()
        self.set_subscriber_protocol()
    
    def set_values(self):
        self.system_mode = 0
        self.planned_velocity = 0
        self.race_mode = 'race'
        self.kiapi_signal = 0
        self.current_location = []
        self.current_velocity = 0
        self.current_heading = 0
        self.planned_route = []
        self.target_actuator = Actuator()
        self.lane_number = 2
        self.curve_list = ['1', '7', '8', '9', '10', '11', '15', '16', '17', '21', '22', '23', '24', 
                           '25', '26', '27', '36', '37', '38', '39', '43', '44', '45', '46', '47', '54', 
                           '59', '60', '61', '62', '63', '68', '69', '70', '72', '73', '78', '79', '80']
        self.curved = False

    def set_publisher_protocol(self):
        self.target_actuator_pub = rospy.Publisher('/control/target_actuator', Actuator, queue_size=1)
        self.lh_test_pub = rospy.Publisher('/lh', Marker, queue_size=1)
    
    def set_subscriber_protocol(self):
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/NavigationData', NavigationData, self.navigation_data_cb)
        rospy.Subscriber('/SystemStatus', SystemStatus, self.system_status_cb)
        rospy.Subscriber('/LaneData', LaneData, self.lane_data_cb)

    def lane_data_cb(self, msg):
        if msg.currentLane.laneNumber.data != 0:
            self.lane_number = msg.currentLane.laneNumber.data
            
        if msg.currentLane.id.data in self.curve_list:
            self.curved = True
        else:
            self.curved = False
        
    
    def system_status_cb(self, msg):
        self.system_mode = msg.systemMode.data 
        self.kiapi_signal = msg.kiapiSignal.data
    
    def navigation_data_cb(self, msg):
        self.planned_velocity = msg.targetVelocity.data
        self.race_mode = msg.raceMode.data
        self.planned_route = []
        for point in msg.plannedRoute:
            self.planned_route.append(Point(x=point.x, y=point.y))
    
    def vehicle_state_cb(self, msg):
        self.current_velocity = msg.velocity.data
        self.current_heading = msg.heading.data
        self.current_location = Point(x=msg.enu.x, y=msg.enu.y)
    
    def publish(self, acc, steer):
        if acc > 0:
            accel = acc
            brake = 0
        else:
            accel = 0
            brake = -acc
        self.target_actuator.steer.data = steer
        self.target_actuator.accel.data = accel
        self.target_actuator.brake.data = brake
        self.target_actuator_pub.publish(self.target_actuator)
    
    def publish_lh(self, point):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.header.frame_id = 'world'
        marker.ns = 'lookahead'
        marker.id = 1
        marker.lifetime = rospy.Duration(0)
        marker.scale.x = 2
        marker.scale.y = 2
        marker.scale.z = 2
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 1
        marker.color.a = 1
        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = 1.0
        self.lh_test_pub.publish(marker)
import rospy
import tf
import tf2_ros

from drive_msgs.msg import *
from visualization_msgs.msg import Marker

from libs.rviz_utils import *

class Visualizer:
    def __init__(self):
        rospy.init_node('visualizer', anonymous=False)

        self.ego_car = CarViz('ego_car', 'ego_car_info', [0, 0, 0], [241, 76, 152, 1])
        self.ego_car_info = CarInfoViz('ego_car', 'ego_car', '',[0,0,0])
        self.br = tf.TransformBroadcaster()

        # calibration
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        static_transforms = [
            ((1.3025, 0.0, 0.0), (0, 0, 0, 1), 'ego_car', 'gps'),   # center
            ((1.1, 0.0, 1.5), (0, 0, 0, 1), 'os_sensor', 'gps'),    # lidar
            ((1.8, 0.0, 1.0), (0, 0, 0, 1), 'front', 'gps'),        # front camera
            #((1.7, 0.7, 1.0), (0, 0, 0, 1), 'left_front', 'gps'),   # left 
            #((1.4, 0.7, 1.0), (0, 0, 0, 1), 'left_rear', 'gps'),    # left
            #((1.7, -0.7, 1.0), (0, 0, 0, 1), 'right_front', 'gps'), # right
            #((1.4, -0.7, 1.0), (0, 0, 0, 1), 'right_rear', 'gps')   # right
        ]
        self.publish_static_tfs(static_transforms)

        self.ego_pos = [0.0, 0.0]
        self.ego_time = rospy.Time(0)

        self.pub_viz_car = rospy.Publisher('/visualizer/car', Marker, queue_size=1)
        self.pub_viz_car_info = rospy.Publisher('/visualizer/car_info', Marker, queue_size=1)
        self.pub_path_viz = rospy.Publisher('/visualizer/local_path', Marker, queue_size=1)
        self.pub_kappa_viz = rospy.Publisher('/visualizer/kappa_viz', Marker, queue_size=1)
        self.pub_objects_viz = rospy.Publisher('/visualizer/objects', MarkerArray, queue_size=1)
        
        rospy.Subscriber('/VehicleState', VehicleState, self.vehicle_state_cb)
        rospy.Subscriber('/NavigationData', NavigationData, self.navigation_data_cb)
        rospy.Subscriber('/DetectionData', DetectionData, self.detection_data_cb)

        rospy.spin()

    def vehicle_state_cb(self, msg):
        yaw = msg.heading.data
        v = msg.velocity.data
        info = f"{(v*3.6):.2f}km/h {yaw:.2f}deg"
        self.ego_car_info.text = info
        quaternion = tf.transformations.quaternion_from_euler(math.radians(0), math.radians(0), math.radians(yaw))  # RPY
        
        if self.ego_time != msg.header.stamp:
            self.br.sendTransform(
                (self.ego_pos[0], self.ego_pos[1], 0),
                (quaternion[0], quaternion[1],quaternion[2], quaternion[3]),
                self.ego_time,'gps','world')
            self.ego_time = msg.header.stamp
        
        self.pub_viz_car.publish(self.ego_car)
        self.pub_viz_car_info.publish(self.ego_car_info)
    
    def navigation_data_cb(self, msg):
        self.ego_pos = [msg.currentLocation.x,msg.currentLocation.y]
        path = []
        for pts in msg.plannedRoute:
            path.append([pts.x, pts.y])
        viz_path = path_viz(path)
        self.pub_path_viz.publish(viz_path)
    
    def detection_data_cb(self, msg):
        objs = []
        for obj in msg.objects:
            objs.append([obj.position.x, obj.position.y, obj.heading.data])
        viz_objects = ObjectsViz(objs)
        self.pub_objects_viz.publish(viz_objects)
    
    # calibration
    def publish_static_tfs(self, transforms):
        static_transformStamped_vec = []
        for translation, rotation, child_frame, parent_frame in transforms:
            static_transformStamped = geometry_msgs.msg.TransformStamped()
            #static_transformStamped.header.stamp = rospy.Time(0)
            static_transformStamped.header.frame_id = parent_frame
            static_transformStamped.child_frame_id = child_frame
            static_transformStamped.transform.translation.x = translation[0]
            static_transformStamped.transform.translation.y = translation[1]
            static_transformStamped.transform.translation.z = translation[2]
            static_transformStamped.transform.rotation.x = rotation[0]
            static_transformStamped.transform.rotation.y = rotation[1]
            static_transformStamped.transform.rotation.z = rotation[2]
            static_transformStamped.transform.rotation.w = rotation[3]
            static_transformStamped_vec.append(static_transformStamped)
        
        self.static_br.sendTransform(static_transformStamped_vec)
        
    
if __name__ == "__main__":
    visualizer = Visualizer()


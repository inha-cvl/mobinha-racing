import rospy
from std_msgs.msg import Bool, Int8

class SensorStatusHandler:
    def __init__(self):
        self.camera_status = True
        self.lidar_status = True
        self.gps_status = True
        self.can_status = True
        
        rospy.Subscriber('/camera_status', Bool, self.camera_status_cb)
        rospy.Subscriber('/lidar_status', Bool, self.lidar_status_cb)
        rospy.Subscriber('/gps_status', Bool, self.gps_status_cb)
        rospy.Subscriber('/can_status', Bool, self.can_status_cb)

    def camera_status_cb(self, msg):
        self.camera_status = msg.cameraStatus.data
    
    def lidar_status_cb(self, msg):
        self.lidar_status = msg.lidarStatus.data
    
    def gps_status_cb(self, msg):
        self.gps_status = msg.gpsStatus.data
    
    def can_status_cb(self, msg):
        self.can_status = msg.canStatus.data

    def check_camera_status(self):
        return self.camera_status

    def check_lidar_status(self):
        return self.lidar_status

    def check_gps_status(self):
        return self.gps_status

    def check_can_status(self):
        return self.can_status
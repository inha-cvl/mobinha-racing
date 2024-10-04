import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Float32MultiArray
from drive_msgs.msg import *
import math
import numpy as np

import tf


class ROSHandler():
    def __init__(self):
        rospy.init_node('perception', anonymous=False)
        self.set_values()
        self.set_protocol()
    
    def set_values(self):
        self.radar_objects = []
        self.est_veh_spd = 0

    def distance(self, x1, y1, x2, y2):
        return np.sqrt((x2-x1)**2+(y2-y1)**2)

    def calculate_radar_heading_velocity(self, rel_vel_x, rel_vel_y):
        heading_radians = math.atan2(rel_vel_x, -rel_vel_y)
        heading_degrees = (math.degrees(heading_radians)+90)
        relative_speed = math.sqrt(rel_vel_x**2 + rel_vel_y**2)
        return heading_degrees, relative_speed

    def cluster_radar_obstacles(self, data, distance_threshold=0.7):
        clusters = []  
        visited = [False] * len(data)  
        for i in range(len(data)):
            if visited[i]:
                continue
            
            x1 = data[i][0]
            y1 = data[i][1]
            cluster = [data[i]]  

            for j in range(i+1, len(data)):
                if visited[j]:
                    continue

                x2 = data[j][0]
                y2 = data[j][1]

                if self.distance(x1, y1, x2, y2) <= distance_threshold:
                    cluster.append(data[j])
                    visited[j] = True
            
            if len(cluster) > 1:
                max_age_point = max(cluster, key=lambda x: x[4])
                heading,velocity = self.calculate_radar_heading_velocity(max_age_point[2], max_age_point[3])
                clusters.append([x1, y1,  heading, velocity, float(max_age_point[4])])
            else:
                heading,velocity = self.calculate_radar_heading_velocity(data[i][2], data[i][3])
                clusters.append([data[i][0], data[i][1], heading, velocity, float(data[i][4])])  
        return clusters

    def set_protocol(self):
        rospy.Subscriber('/RadarObjectArray', RadarObjectArray, self.radar_object_array_cb)
        rospy.Subscriber('/ADAS_DRV',Float32MultiArray, self.adas_drv_cb )
        self.radar_objects_marker_pub = rospy.Publisher('/RadarObjects', MarkerArray, queue_size=1)
    
    def adas_drv_cb(self, msg):
        self.est_veh_spd = msg.data[5]

    def radar_object_array_cb(self, msg):
        radar_objects = []
        for ro in msg.radarObjects:
            if ro.mvngFlag.data > 0 and ro.qualLvl.data > 33  and ro.coastAge.data < 1 and ro.alvAge.data > 10:
                obj = [ro.relPosX.data+2.325, ro.relPosY.data, ro.relVelX.data, ro.relVelY.data, ro.alvAge.data]
                radar_objects.append(obj)
        clustered_objects = self.cluster_radar_obstacles(radar_objects)
        self.radar_objects = clustered_objects

    def publish(self, positions):
        
        marker_array = MarkerArray()

        for i, (x, y, o1, o2, o3) in enumerate(positions):

            marker = Marker()
            marker.header.frame_id = "hesai_lidar"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration(0.05)
            marker.pose.position.x = x+0.8
            marker.pose.position.y = y
            marker.pose.position.z = -1.2  # Assume ground level for simplicity
            quaternion = tf.transformations.quaternion_from_euler(0, 0, math.radians(o1))
            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]
            marker.pose.orientation.w = quaternion[3]
            marker.scale.x = 1.6
            marker.scale.y = 1.6
            marker.scale.z = 1.6
            marker.color.a = 0.5  # Don't forget to set the alpha!
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)

            marker = Marker()
            marker.header.frame_id = "hesai_lidar"
            marker.ns = "objects"
            marker.id = i+32
            marker.type = Marker.TEXT_VIEW_FACING
            marker.lifetime = rospy.Duration(0.05)
            marker.scale.z = 1
            marker.color.r = 1
            marker.color.g = 1
            marker.color.b = 1
            marker.color.a = 1.0
            marker.pose.position.x = x+0.8
            marker.pose.position.y = y
            marker.pose.position.z = -1.2+0.65
            marker.text = f"{(x-2.325):.1f}m {o2:.1f}m/s"
            #°
            marker_array.markers.append(marker)

        self.radar_objects_marker_pub.publish(marker_array)

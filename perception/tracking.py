import tf.transformations
from obstacles_info import Object
from numpy.linalg import inv
from visualization_msgs.msg import Marker, MarkerArray
from scipy.optimize import linear_sum_assignment
from collections import deque
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox
import copy
import time
import math
import tf
import numpy as np
import rospy
import math
import datetime as dt
from geometry_msgs.msg import Point
from ublox_msgs.msg import NavPVT

class ObjectTracker():
    def __init__(self,  remove_count=4, distance_threshold=2.5, appear_count=2):
        self.next_object_id = 0
        self.prev_objects = {}  
        self.disappeared = {}  
        self.candidate_objects = {} 
        self.candidate_next_object_id = 100000  
        self.candidate_alive_count = {}  # 후보 객체들의 alivecount

        self.remove_count = remove_count
        self.distance_threshold = distance_threshold
        self.appear_count = appear_count  # alivecount가 이 값 이상이면 tracking on!

        # ego ego
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        self.gps_buffer = deque(maxlen=10)
        self.bboxarray = None
        self.gps_msg = None
        self.gspeed = None

        self.prev_bbox_timestamp = None
        self.dt = None  # dt를 저장할 변수

        rospy.Subscriber("/ublox/navpvt", NavPVT, self.gspeed_callback)
        #rospy.Subscriber('/deep_box', BoundingBoxArray, self.bbox_callback)
        # rospy.Subscriber('/best/poseStamped', PoseStamped, self.gps_callback)

        rospy.Subscriber("/mobinha/perception/lidar/track_box", BoundingBoxArray, self.bbox_callback)

        self.tracker_pub = rospy.Publisher("/tracked_marker", BoundingBoxArray, queue_size=10)
        self.footprint_pub = rospy.Publisher("/foot_print", MarkerArray, queue_size=10)
        self.text_pub = rospy.Publisher("/text", MarkerArray, queue_size=10)
        self.x_list = []
        self.y_list = []
        self.i = 0

    def gps_callback(self, gps_msg):
        self.gps_buffer.append(gps_msg)

    def gspeed_callback(self, nav_msg):
        self.gspeed = (nav_msg.gSpeed / 1000.0) * 3.6

    def bbox_callback(self, bbox_msg):
        self.bboxarray = bbox_msg
        # if not self.gps_buffer:
        #     rospy.logwarn("No gps_msg available for bbox_msg at time {}".format(bbox_msg.header.stamp.to_sec()))
        #     return

        # closest_gps_msg = min(self.gps_buffer, key=lambda gps_msg: abs((gps_msg.header.stamp - bbox_msg.header.stamp).to_sec()))

        # self.gps_msg = closest_gps_msg
        # self.x = self.gps_msg.pose.position.x
        # self.y = self.gps_msg.pose.position.y
        # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([self.gps_msg.pose.orientation.x,
        #                                                                self.gps_msg.pose.orientation.y,
        #                                                                self.gps_msg.pose.orientation.z,
        #                                                                self.gps_msg.pose.orientation.w])
        # self.heading = math.degrees(yaw)

        if self.prev_bbox_timestamp is not None:
            self.dt = (bbox_msg.header.stamp - self.prev_bbox_timestamp).to_sec()
            # print("dt:", self.dt)
        else:
            self.dt = 0.05  # 첫 번째 메시지인 경우 dt를 일단은 1/20로 해두자 

        # 현재 bbox_msg의 타임스탬프를 저장하여 
        self.prev_bbox_timestamp = bbox_msg.header.stamp

        self.update()

    def transfer(self, object_x, object_y):
        rad = np.radians(self.heading)
        rotated_x = (object_x) * math.cos(rad) - object_y * math.sin(rad)
        rotated_y = (object_x) * math.sin(rad) + object_y * math.cos(rad)
        enu_pos_x = rotated_x + self.x
        enu_pos_y = rotated_y + self.y
        return enu_pos_x, enu_pos_y

    def push_back(self, object):
        self.prev_objects[self.next_object_id] = object
        self.disappeared[self.next_object_id] = 0
        self.next_object_id += 1

    def remove(self, object_id):
        del self.prev_objects[object_id]
        del self.disappeared[object_id]

    def remove_candidate(self, object_id):
        del self.candidate_objects[object_id]
        del self.candidate_alive_count[object_id]

    def distance(self, prev_object, bbox):
        prev_center_x, prev_center_y = prev_object.state_est[0], prev_object.state_est[1]
        new_center_x, new_center_y = bbox.bbox[0], bbox.bbox[1]
        distance = math.sqrt((prev_center_x - new_center_x) ** 2 + (prev_center_y - new_center_y) ** 2)
        return distance

    def size(self, bbox):
        size = bbox.bbox[3] * bbox.bbox[4] * bbox.bbox[5]
        return size

    def update(self):
        # if self.bboxarray is not None and self.gps_msg is not None :
        if self.bboxarray is not None:
            start = time.time()
            bboxes = []
            for box in self.bboxarray.boxes:
                if box.label == 1:
                    center_x = box.pose.position.x
                    center_y = box.pose.position.y
                    center_z = 0.0
                    scale_x = box.dimensions.x
                    scale_y = box.dimensions.y
                    scale_z = box.dimensions.z
                    bbox = [center_x, center_y, center_z, scale_x, scale_y, scale_z]
                    object = Object(bbox)
                    bboxes.append(object)

            # matching
            object_ids = list(self.prev_objects.keys())
            cost_matrix = np.zeros((len(self.prev_objects), len(bboxes)))

            for i in range(len(object_ids)):
                for j in range(len(bboxes)):
                    distance = self.distance(self.prev_objects[object_ids[i]], bboxes[j])
                    if distance <= self.distance_threshold:
                        cost_matrix[i, j] = distance
                    else:
                        cost_matrix[i, j] = 1e5

            row_ind, col_ind = linear_sum_assignment(cost_matrix)

            unmatched_rows = set(range(cost_matrix.shape[0]))
            unmatched_cols = set(range(cost_matrix.shape[1]))

            for row, col in zip(row_ind, col_ind):
                if cost_matrix[row, col] <= self.distance_threshold:
                    object_id = object_ids[row]
                    self.prev_objects[object_id].update(bboxes[col].bbox[0], bboxes[col].bbox[1], self.dt)
                    self.disappeared[object_id] = 0
                    unmatched_rows.discard(row)
                    unmatched_cols.discard(col)

            # unmatched candidate
            for row in unmatched_rows:
                object_id = object_ids[row]
                self.disappeared[object_id] += 1
                self.prev_objects[object_id].predict(self.dt)
                # if object_id == 0:    
                #     print("object id : {0} is predicted!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!".format(object_id))

                if self.disappeared[object_id] > self.remove_count:
                    self.remove(object_id)

            candidate_ids = list(self.candidate_objects.keys())
            candidate_cost_matrix = np.zeros((len(candidate_ids), len(unmatched_cols)))

            for i in range(len(candidate_ids)):
                for j, col in enumerate(unmatched_cols):
                    distance = self.distance(self.candidate_objects[candidate_ids[i]], bboxes[col])
                    if distance <= self.distance_threshold:
                        candidate_cost_matrix[i, j] = distance
                    else:
                        candidate_cost_matrix[i, j] = 1e5

            candidate_row_ind, candidate_col_ind = linear_sum_assignment(candidate_cost_matrix)

            unmatched_candidate_rows = set(range(candidate_cost_matrix.shape[0]))
            unmatched_candidate_cols = set(range(candidate_cost_matrix.shape[1]))

            matched_cols = set()
            for row, col in zip(candidate_row_ind, candidate_col_ind):
                if candidate_cost_matrix[row, col] <= self.distance_threshold:
                    candidate_id = candidate_ids[row]
                    col = list(unmatched_cols)[col]
                    self.candidate_objects[candidate_id].update(bboxes[col].bbox[0], bboxes[col].bbox[1], self.dt)
                    self.candidate_alive_count[candidate_id] += 1
                    unmatched_candidate_rows.discard(row)
                    unmatched_candidate_cols.discard(col)
                    matched_cols.add(col)

                    if self.candidate_alive_count[candidate_id] >= self.appear_count:
                        self.push_back(self.candidate_objects[candidate_id])
                        self.remove_candidate(candidate_id)

            # no matching handling
            for row in unmatched_candidate_rows:
                candidate_id = candidate_ids[row]
                self.candidate_alive_count[candidate_id] -= 1
                self.candidate_objects[candidate_id].predict(self.dt)

                if self.candidate_alive_count[candidate_id] <= 0:
                    self.remove_candidate(candidate_id)

            # unmatched object pushback
            new_unmatched_cols = unmatched_cols - matched_cols
            for col in new_unmatched_cols:
                candidate_id = self.candidate_next_object_id
                self.candidate_objects[candidate_id] = bboxes[col]
                self.candidate_alive_count[candidate_id] = 1
                self.candidate_next_object_id += 1

            result = self.publish_markers(self.prev_objects)
            self.tracker_pub.publish(result)
            self.text_pub.publish(self.publish_text(self.prev_objects))
            end = time.time()

    def publish_text(self, objects):
        text_array = MarkerArray()
        for object_id in objects.keys():
            bbox = objects[object_id]
            center_x = bbox.state_est[0]
            center_y = bbox.state_est[1]
            center_z = bbox.bbox[2]
            text = Marker()
            text.id = object_id
            text.header.frame_id = "hesai_lidar"
            text.type = Marker.TEXT_VIEW_FACING
            text.text = "object_id : {0}, velocity : {1}".format(object_id, round(bbox.state_est[2] * 3.6, 2))
            text.action = Marker.ADD
            text.pose.position.x = center_x
            text.pose.position.y = center_y
            text.pose.position.z = center_z + 0.5
            text.scale.z = 2
            text.color.a = 1.0
            text.color.r = 1.0
            text.color.g = 0.0
            text.color.b = 0.0
            text.lifetime = rospy.Duration(0.1)
            text_array.markers.append(text)
        return text_array

    def publish_markers(self, objects):
        bbox_array = BoundingBoxArray()
        bbox_array.header.frame_id = "hesai_lidar"
        bbox_array.header.stamp = rospy.Time.now()

        for object_id in objects.keys():
            bbox = objects[object_id]
            center_x = bbox.state_est[0]
            center_y = bbox.state_est[1]
            center_z = bbox.bbox[2]
            scale_x = bbox.bbox[3]
            scale_y = bbox.bbox[4]
            scale_z = bbox.bbox[5]

            # BoundingBox message for each tracked object
            bounding_box = BoundingBox()
            bounding_box.header.frame_id = "hesai_lidar"
            bounding_box.header.stamp = rospy.Time.now()
            bounding_box.pose.position.x = center_x
            bounding_box.pose.position.y = center_y
            bounding_box.pose.position.z = 0  # ground level
            bounding_box.dimensions.x = scale_x
            bounding_box.dimensions.y = scale_y
            bounding_box.dimensions.z = scale_z
            bounding_box.value = round(bbox.state_est[2], 2)
            bbox_array.boxes.append(bounding_box)

        return bbox_array

if __name__ == "__main__":
    rospy.init_node('tracker')
    tracker = ObjectTracker()
    rospy.spin()

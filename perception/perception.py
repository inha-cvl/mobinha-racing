import rospy
import sys
import signal
import time
import cv2
import numpy as np
import math

from ros_handler import ROSHandler
import perception_handler as ph


M_PI = math.pi / 180.0

def signal_handler(sig, frame):
    sys.exit(0)

class Perception():
    def __init__(self):
        self.RH = ROSHandler()
        self.set_values()
        self.next_id = 0  # 객체 ID 생성용 변수
    
    def set_values(self):
        self.intrinsic = np.array([[906.34269, 0.000000e+00, 798.47982], 
                                   [0.000000e+00, 905.27766, 550.20956],
                                   [0.000000e+00, 0.000000e+00, 1.000000e+00]])

        self.dist_coeffs = np.array([-0.13287, 0.08114, 0.00005, 0.00026, 0.00000])
        self.roll, self.pitch, self.yaw = 90.4, 1.2, 89.5
        self.tx, self.ty, self.tz = -0.005, 1.2, 1.6
        self.increment = 0.1
        self.old_matched_boxes = []
        self.disappeared = {}
        self.max_disappear = 3
        self.dt = 0.05
        self.distance_threshold = 1.0

    def update_disappeared(self):
        for i, old_box in enumerate(self.old_matched_boxes):
            box_id = old_box[2]  # ID는 2번째 요소에 저장
            if box_id in self.disappeared:
                self.disappeared[box_id] += 1
            else:
                self.disappeared[box_id] = 1
            
            if self.disappeared[box_id] <= self.max_disappear:
                position_info = list(old_box[1])
                position_info[0] += position_info[4] * self.dt  
                position_info[1] += position_info[4] * self.dt 
                self.old_matched_boxes[i] = (old_box[0], tuple(position_info), old_box[2])  # ID 유지
            else:
                del self.disappeared[box_id]
                self.old_matched_boxes.pop(i)
                
    def is_similar_position(self, old_box, new_box):
        old_position_x, old_position_y = old_box[1][0], old_box[1][1]
        new_position_x, new_position_y = new_box[1][0], new_box[1][1]

        distance = np.sqrt((new_position_x - old_position_x) ** 2 + (new_position_y - old_position_y) ** 2)

        return distance <= self.distance_threshold

    def transformation(self):
        R = np.array([[1.0, 0.0, 0.0],
                      [0.0, math.cos(self.roll * M_PI), -math.sin(self.roll * M_PI)],
                      [0.0, math.sin(self.roll * M_PI), math.cos(self.roll * M_PI)]])
        
        P = np.array([[math.cos(self.pitch * M_PI), 0.0, math.sin(self.pitch * M_PI)],
                      [0.0, 1.0, 0.0],
                      [-math.sin(self.pitch * M_PI), 0.0, math.cos(self.pitch * M_PI)]])
        
        Y = np.array([[math.cos(self.yaw * M_PI), -math.sin(self.yaw * M_PI), 0.0],
                      [math.sin(self.yaw * M_PI), math.cos(self.yaw * M_PI), 0.0],
                      [0.0, 0.0, 1.0]])
        
        return np.dot(R, np.dot(P, Y))
    
    def execute(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.RH.img is None:
                continue
            
            start = time.time()
            img_undistorted = self.RH.img.copy()
            
            clustered_list = ph.cluster_radar_obstacles(self.RH.radar_objects)            
            points = ph.find_corners(clustered_list)

            R = self.transformation()
            T = np.array([self.tx, self.ty, self.tz]).reshape(3, 1)
            P = np.hstack((R, T))
                    
            points_2d = ph.rectify_corners(points, P, self.intrinsic)
            matched_boxes = ph.match_bounding_boxes(points_2d, self.RH.bounding_boxes)
            
            updated_old_boxes = [] 
            used_new_boxes = set() 
            for i, old_box in enumerate(self.old_matched_boxes):
                found_match = False
                for j, new_box in enumerate(matched_boxes):
                    if self.is_similar_position(old_box, new_box):
                        if old_box[2] not in self.disappeared:  # old_box[2]는 객체 ID
                            self.disappeared[old_box[2]] = 0  
                        self.disappeared[old_box[2]] = 0  
                        old_position_info = list(old_box[1])  
                        old_position_info[0] = new_box[1][0]  
                        old_position_info[1] = new_box[1][1]  
                        old_position_info[4] = new_box[1][4]  
                        updated_old_boxes.append((old_box[0], tuple(old_position_info), old_box[2]))  # ID 유지
                        used_new_boxes.add(j)
                        found_match = True
                        break
                
                if not found_match:
                    if old_box[2] not in self.disappeared:
                        self.disappeared[old_box[2]] = 0  
                    if self.disappeared[old_box[2]] < self.max_disappear:
                        updated_old_boxes.append(old_box)
                    self.disappeared[old_box[2]] += 1  

            for j, new_box in enumerate(matched_boxes):
                if j not in used_new_boxes: 
                    # 새로운 박스에 ID 부여
                    new_id = self.next_id
                    self.next_id += 1
                    updated_old_boxes.append((new_box[0], new_box[1], new_id))
                    self.disappeared[new_id] = 0  

            self.old_matched_boxes = updated_old_boxes

            matched_list = []
            for bbox, info, obj_id in self.old_matched_boxes:
                xmin, ymin, xmax, ymax = bbox
                posx, posy, posz = info[0:3]
                heading, velocity = info[3:5]
                matched_list.append((posx, posy, heading, velocity, obj_id))  
                cv2.rectangle(img_undistorted, (xmin, ymin), (xmax, ymax), (255, 255, 255), 2)
                cv2.putText(img_undistorted, f"x={posx:.2f} y={posy:.2f}", (xmin, ymin), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            
            self.RH.publish_result_img(img_undistorted)
            self.RH.publish(self.RH.radar_objects)
            self.RH.publish_object_array(matched_list)
            rate.sleep()



def main():
    signal.signal(signal.SIGINT, signal_handler)
    perception = Perception()
    perception.execute()

if __name__ == "__main__":
    main()
import rospy
import sys
import signal
from ros_handler import ROSHandler
import time
import math
import numpy as np
from scipy.optimize import linear_sum_assignment
from shapely.geometry import Polygon
from local_path_test import LocalPathTest
from hd_map.map import MAP
from libs.lanelet_handler import LaneletHandler

def signal_handler(sig, frame):
    sys.exit(0)

class MapLane():
    def __init__(self):
        self.RH = ROSHandler()
        self.map = None
        self.lpt = None
        self.set_values()

    
    def set_values(self):
        self.lpt_use = False
        self.max_vel = 30
        self.stacked_refine_obstacles = []
        self.obstacle_timestamps = []
        self.remian_duration = 2

    def map_publish(self):
        self.RH.publish_map_viz(self.map.lmap_viz, self.map.mlmap_viz)
    
    def map_initialize(self):
        if self.RH.map_name != None:
            self.map = MAP(self.RH.map_name)
            self.llh = LaneletHandler(self.RH, self.map)
            if self.lpt_use:
                self.lpt = LocalPathTest(self.RH, self.map)
            
    def update_obstacles(self, new_obstacles):
        current_time = time.time()
        if len(new_obstacles) > 0:
            self.stacked_refine_obstacles.extend(new_obstacles)
            self.obstacle_timestamps.append(current_time)
            
        self.stacked_refine_obstacles = [
            obs for obs, timestamp in zip(self.stacked_refine_obstacles, self.obstacle_timestamps) 
            if current_time - timestamp < self.remian_duration
        ]
        self.obstacle_timestamps = [
            timestamp for timestamp in self.obstacle_timestamps 
            if current_time - timestamp < self.remian_duration
        ]
    
    
    def rotate_point(self, px, py, cx, cy, angle):

        sin_theta = math.sin(angle)
        cos_theta = math.cos(angle)
        
        translated_x = px - cx
        translated_y = py - cy
        
        rotated_x = translated_x * cos_theta - translated_y * sin_theta + cx
        rotated_y = translated_x * sin_theta + translated_y * cos_theta + cy
        
        return rotated_x, rotated_y



    def calculate_iou(self, obj1, obj2, size_x, size_y):
        x1, y1, heading1 = obj1[1], obj1[2], obj1[4]
        x2, y2, heading2 = obj2[1], obj2[2], obj2[4]

        corners1 = [
            (x1 - size_x / 2, y1 - size_y / 2),
            (x1 + size_x / 2, y1 - size_y / 2),
            (x1 + size_x / 2, y1 + size_y / 2),
            (x1 - size_x / 2, y1 + size_y / 2)
        ]

        corners2 = [
            (x2 - size_x / 2, y2 - size_y / 2),
            (x2 + size_x / 2, y2 - size_y / 2),
            (x2 + size_x / 2, y2 + size_y / 2),
            (x2 - size_x / 2, y2 + size_y / 2)
        ]

        rotated_corners1 = [self.rotate_point(px, py, x1, y1, heading1) for (px, py) in corners1]
        rotated_corners2 = [self.rotate_point(px, py, x2, y2, heading2) for (px, py) in corners2]

        poly1 = Polygon(rotated_corners1)
        poly2 = Polygon(rotated_corners2)

        inter_area = poly1.intersection(poly2).area
        
        box1_area = poly1.area
        box2_area = poly2.area
        
        iou = inter_area / (box1_area + box2_area - inter_area)
        return iou

    def lidar_radar_matching(self, lidar_objects, radar_objects,iou_threshold = 0.1, size_x = 4.365, size_y = 1.85):
        iou_matrix = np.zeros((len(lidar_objects), len(radar_objects)))

        for i, lidar_obj in enumerate(lidar_objects):
            for j, radar_obj in enumerate(radar_objects):
                iou = self.calculate_iou(lidar_obj, radar_obj, size_x, size_y)
                if iou >= iou_threshold:
                    iou_matrix[i, j] = iou
                else:
                    iou_matrix[i, j] = -10e5

        row_ind, col_ind = linear_sum_assignment(-iou_matrix)

        print("radar objects : ", radar_objects)
        print("lidar objects : " , lidar_objects)
        matched_pairs = list(zip(row_ind, col_ind))
        matched_pairs_filtered = []
        # lidar_matched = []
        radar_matched = []
        # combined_object = []
        for l, r in matched_pairs:
            if iou_matrix[l][r] >= iou_threshold:
                lidar_objects[l][3] = radar_objects[r][3]
                matched_pairs_filtered.append((l, r))
                # lidar_matched.append(l)
                radar_matched.append(r)
        
        
        print(matched_pairs_filtered)
        # lidar_matched.sort(reverse=True)
        # for index in lidar_matched:
        #     del lidar_objects[index]
        # matched_pairs_filtered.append(lidar_objects)
            
        radar_matched.sort(reverse=True)
        for index in radar_matched:
            del radar_objects[index]

        for i in range(len(lidar_objects)):
            lidar_objects[i][0] = 1

        for i , radar_object in enumerate(radar_objects):
            radar_object[0] = 2
            lidar_objects.append(radar_object)

        print("final objects :",lidar_objects)
        return lidar_objects, matched_pairs_filtered
        

    def execute(self):
        while self.map == None:
            self.map_initialize()
        self.map_publish()

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.lpt is None:
                pass
            if self.lpt_use:
                lp_result = self.lpt.execute(self.RH.local_pose)
                if lp_result == None:
                    local_path, local_kappa = None, None 
                else:
                    [ local_path, local_kappa ] = lp_result
                local_velocity = self.lpt.get_velocity(self.max_vel)
                self.RH.publish(local_path, local_kappa, local_velocity)
            
            refine_obstacles = self.llh.refine_obstacles_heading(self.RH.local_pose, [self.RH.sim_obstacles, self.RH.cam_obstacles, self.RH.lid_obstacles, self.RH.rad_obstacles, self.RH.fus_obstacles])
            
            lidar_obstacles = self.llh.refine_obstacles_heading(self.RH.local_pose, [self.RH.lid_obstacles])
            radar_obstacles = self.llh.refine_obstacles_heading(self.RH.local_pose, [self.RH.fus_obstacles])
            refine_obstacles = self.lidar_radar_matching(lidar_obstacles, radar_obstacles)
            
            self.update_obstacles(refine_obstacles)
            #self.RH.publish_refine_obstacles(self.stacked_refine_obstacles)    
            self.RH.publish_refine_obstacles(refine_obstacles)
            
            lane_data, lane_id = self.llh.get_lane_number(self.RH.local_pose)

            if lane_data is None and lane_id is None:
                curr_lane_num = None
                curr_lane_id = None
            else:
                curr_lane_num = lane_data
                curr_lane_id = lane_id
            self.RH.publish_lane_data(curr_lane_num, curr_lane_id)
            rate.sleep()
                

def main():
    signal.signal(signal.SIGINT, signal_handler)
    map_lane = MapLane()
    map_lane.execute()

if __name__ == "__main__":
    main()
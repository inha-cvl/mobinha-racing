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
    
    def set_values(self):
        self.intrinsic = np.array([[906.34269, 0.000000e+00, 798.47982], 
                                   [0.000000e+00, 905.27766, 550.20956],
                                   [0.000000e+00, 0.000000e+00, 1.000000e+00]])

        # self.intrinsic = np.array([[898.92473, 0.000000e+00, 798.47982],  
        #                            [0.000000e+00, 896.90500, 550.20956],  
        #                            [0.000000e+00, 0.000000e+00, 1.000000e+00]])  

        self.dist_coeffs = np.array([-0.13287, 0.08114, 0.00005, 0.00026, 0.00000])

        self.roll, self.pitch, self.yaw = 90.4, 1.2, 90.0
        self.tx, self.ty, self.tz = -0.005, 1.2, 1.6
        self.increment = 0.1

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
            # img_undistorted = cv2.undistort(self.RH.img, self.intrinsic, self.dist_coeffs)
            
            #radar clustering
            # clustered_list = ph.cluster_radar_obstacles(self.RH.radar_objects)            
            #find corners
            points = ph.find_corners(self.RH.radar_objects)

            R = self.transformation()
            T = np.array([self.tx, self.ty, self.tz]).reshape(3, 1)
            P = np.hstack((R, T))
                    
            points_2d = ph.rectify_corners(points, P, self.intrinsic)
            matched_boxes = ph.match_bounding_boxes(points_2d, self.RH.bounding_boxes)
            for (xmin, ymin, xmax, ymax) in self.RH.bounding_boxes:
                cv2.rectangle(img_undistorted, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)
                
            # for (xmin, ymin, xmax, ymax, *rest) in points_2d:
            #     cv2.rectangle(img_undistorted, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

            matched_list = []
            for bbox, info in matched_boxes:
                xmin, ymin, xmax, ymax = bbox
                posx, posy, posz = info[0:3]
                heading, velocity = info[3:5]
                matched_list.append((posx, posy, heading, velocity))
                cv2.rectangle(img_undistorted, (xmin, ymin), (xmax, ymax), (255, 255, 255), 2)
                cv2.putText(img_undistorted, f"x={posx:.2f} y={posy:.2f}", (xmin, ymin), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            
            #print("Radar + Camera Fusion time: ", time.time() - start)
            
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
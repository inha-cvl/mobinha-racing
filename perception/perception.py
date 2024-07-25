import rospy
import sys
import signal
import numpy as np
import cv2
from copy import deepcopy

from ros_handler import ROSHandler

def signal_handler(sig, frame):
    sys.exit(0)

class Perception():
    def __init__(self):
        self.RH = ROSHandler()
        self.set_values()

    def set_values(self):
        self.focal_length = 903.89587 
        self.box_width = 0.5  
        self.camera_height = 1.45  

    def detect_boxes(self, hsv):
        h, s, v = cv2.split(hsv)
        v = cv2.equalizeHist(v)
        hsv = cv2.merge((h, s, v))
        
        green_lower = np.array([0, 60, 210])
        green_upper = np.array([180, 255, 240])
        
        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        return contours

    def calculate_distance(self, contour, known_width):
        x, y, w, h = cv2.boundingRect(contour)
        distance_2d = (known_width * self.focal_length) / w
        return distance_2d, (x + w // 2, y + h // 2)

    def calculate_3d_distance(self, distance_2d):
        distance_3d = np.sqrt(distance_2d**2 + self.camera_height**2)
        return distance_3d

    def calculate_object_position(self, distance_2d, center, image_center):
        dx = center[0] - image_center[0]
        y = (dx * distance_2d) / self.focal_length  # y 값 계산, 왼쪽이 음수, 오른쪽이 양수
        x = distance_2d  # x 값은 distance_2d
        return x, y

    def edge_enhance(self, frame):
        kernel = np.array([[0, -1, 0],
                           [-1, 5, -1],
                           [0, -1, 0]])
        enhanced_frame = cv2.filter2D(frame, -1, kernel)
        return enhanced_frame

    def execute(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.RH.frame is None:
                continue
            frame_copy = deepcopy(self.RH.frame)
            hsv = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2HSV)
            hsv_sharpe = self.edge_enhance(hsv)
            contours = self.detect_boxes(hsv_sharpe)
            h, w, _ = frame_copy.shape
            image_center = (w // 2, h // 2)
            
            positions = []
            for contour in contours:
                c_area = cv2.contourArea(contour)
                if 80 < c_area < 1000:
                    distance_2d, center = self.calculate_distance(contour, self.box_width)
                    if w * 0.45 < center[0] < w * 0.55:
                        distance_3d = self.calculate_3d_distance(distance_2d)
                        object_x, object_y = self.calculate_object_position(distance_3d, center, image_center)
                        positions.append((object_x, object_y))
                        
                        cv2.putText(frame_copy, f"Dist: {object_x:.2f}m, Y: {object_y:.2f}m", center, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.drawContours(frame_copy, [contour], -1, (255, 255, 255), 2)

            self.RH.publish(frame_copy, positions)
            rate.sleep()

def main():
    signal.signal(signal.SIGINT, signal_handler)
    perception = Perception()
    perception.execute()

if __name__ == "__main__":
    main()






# import rospy
# import sys
# import signal
# import numpy as np
# import cv2
# from copy import deepcopy

# from ros_handler import ROSHandler

# def signal_handler(sig, frame):
#     sys.exit(0)

# class Perception():
#     def __init__(self):
#         self.RH = ROSHandler()
#         self.set_values()
#         self.tracker_initialized = False
#         self.tracker = self.create_tracker()
#         self.bbox = None

#     def set_values(self):
#         self.focal_length = 903.89587 
#         self.box_width = 0.5  
#         self.camera_height = 1.45  

#     def create_tracker(self):
#         # Create the tracker based on the OpenCV version
#         try:
#             if hasattr(cv2, 'legacy'):
#                 if hasattr(cv2.legacy, 'TrackerCSRT_create'):
#                     return cv2.legacy.TrackerCSRT_create()
#                 elif hasattr(cv2.legacy, 'TrackerKCF_create'):
#                     return cv2.legacy.TrackerKCF_create()
#             else:
#                 if hasattr(cv2, 'TrackerCSRT_create'):
#                     return cv2.TrackerCSRT_create()
#                 elif hasattr(cv2, 'TrackerKCF_create'):
#                     return cv2.TrackerKCF_create()
#                 elif hasattr(cv2, 'TrackerMIL_create'):
#                     return cv2.TrackerMIL_create()
#                 elif hasattr(cv2, 'TrackerMedianFlow_create'):
#                     return cv2.TrackerMedianFlow_create()
#             rospy.logerr("No suitable tracker found in your OpenCV version.")
#             sys.exit(1)
#         except AttributeError as e:
#             rospy.logerr("Error creating tracker: {}".format(e))
#             sys.exit(1)



#     def detect_boxes(self, hsv):
#         h, s, v = cv2.split(hsv)
#         v = cv2.equalizeHist(v)
#         hsv = cv2.merge((h, s, v))
        
#         green_lower = np.array([0, 50, 210])
#         green_upper = np.array([180, 255, 240])
        
#         green_mask = cv2.inRange(hsv, green_lower, green_upper)
#         contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
#         return contours

#     def calculate_distance(self, contour, known_width):
#         x, y, w, h = cv2.boundingRect(contour)
#         distance_2d = (known_width * self.focal_length) / w
#         return distance_2d, (x + w // 2, y + h // 2)

#     def calculate_3d_distance(self, distance_2d):
#         distance_3d = np.sqrt(distance_2d**2 + self.camera_height**2)
#         return distance_3d

#     def calculate_object_position(self, distance_2d, center, image_center):
#         dx = center[0] - image_center[0]
#         y = (dx * distance_2d) / self.focal_length  # y 값 계산, 왼쪽이 음수, 오른쪽이 양수
#         x = distance_2d  # x 값은 distance_2d
#         return x, y

#     def edge_enhance(self, frame):
#         kernel = np.array([[0, -1, 0],
#                            [-1, 5, -1],
#                            [0, -1, 0]])
#         enhanced_frame = cv2.filter2D(frame, -1, kernel)
#         return enhanced_frame

#     def initialize_tracker(self, frame, contour):
#         x, y, w, h = cv2.boundingRect(contour)
#         self.bbox = (x, y, w, h)
#         self.tracker.init(frame, self.bbox)
#         self.tracker_initialized = True

#     def update_tracker(self, frame):
#         success, bbox = self.tracker.update(frame)
#         if success:
#             x, y, w, h = [int(v) for v in bbox]
#             center = (x + w // 2, y + h // 2)
#             return success, center, w
#         else:
#             return success, None, None

#     def execute(self):
#         rate = rospy.Rate(20)
#         while not rospy.is_shutdown():
#             if self.RH.frame is None:
#                 continue
#             frame_copy = deepcopy(self.RH.frame)
#             hsv = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2HSV)
#             hsv_sharpe = self.edge_enhance(hsv)
#             contours = self.detect_boxes(hsv_sharpe)
#             h, w, _ = frame_copy.shape
#             image_center = (w // 2, h // 2)

#             if not self.tracker_initialized:
#                 for contour in contours:
#                     c_area = cv2.contourArea(contour)
#                     if 100 < c_area < 1000:
#                         distance_2d, center = self.calculate_distance(contour, self.box_width)
#                         if w * 0.45 < center[0] < w * 0.55:
#                             self.initialize_tracker(frame_copy, contour)
#                             break

#             if self.tracker_initialized:
#                 success, center, bbox_width = self.update_tracker(frame_copy)
#                 if success:
#                     distance_2d = (self.box_width * self.focal_length) / bbox_width
#                     distance_3d = self.calculate_3d_distance(distance_2d)
#                     object_x, object_y = self.calculate_object_position(distance_2d, center, image_center)
                    
#                     cv2.putText(hsv_sharpe, f"Dist: {object_x:.2f}m, Y: {object_y:.2f}m", center, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
#                     cv2.rectangle(hsv_sharpe, (int(center[0] - bbox_width/2), int(center[1] - bbox_width/2)),
#                                   (int(center[0] + bbox_width/2), int(center[1] + bbox_width/2)), (255, 255, 255), 2)
#                 else:
#                     self.tracker_initialized = False

#             self.RH.publish(hsv_sharpe)
#             rate.sleep()

# def main():
#     signal.signal(signal.SIGINT, signal_handler)
#     perception = Perception()
#     perception.execute()

# if __name__ == "__main__":
#     main()



'''
# gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)
            # sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
            # sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
            # sobel_combined = cv2.addWeighted(cv2.convertScaleAbs(sobel_x), 1, cv2.convertScaleAbs(sobel_y), 0.5, 0)
            # blurred = cv2.GaussianBlur(sobel_combined, (5, 5), 0)
            # edges = cv2.Canny(blurred, 50, 150)

            # # Find contours
            # contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # for contour in contours:
            #     # Approximate the contour
            #     approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
            #     if len(approx) > 4:  # We assume that the box has 4 edges
            #         # Compute the bounding box
            #         (x, y, w, h) = cv2.boundingRect(approx)
            #         # Draw the bounding box
            #         cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (0, 255, 0), 2)

            #         # Estimate distance
            #         focal_length = 800  # Example focal length in pixels
            #         real_box_height = 0.5  # Example real height of the box in meters
            #         distance = (real_box_height * focal_length) / h

            #         cv2.putText(frame_copy, f"Distance: {distance:.2f} m", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # #g_to_color = cv2.cvtColor(frame_copy, cv2.COLOR_GRAY2RGB)
'''
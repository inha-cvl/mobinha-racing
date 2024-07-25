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
        self.white_box_width = 0.5 
        self.green_box_width = 1 
        self.camera_height = 1.45  
        self.num_frames = 10
        self.left_lines = []
        self.right_lines = []

    def detect_boxes(self, hsv):
        h, s, v = cv2.split(hsv)
        v = cv2.equalizeHist(v)
        hsv = cv2.merge((h, s, v))
        
        white_lower = np.array([90, 20,220])
        white_upper = np.array([120,100,255])
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        green_lower = np.array([60, 40, 0])
        green_upper = np.array([80, 255, 255])
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        kernel = np.ones((2, 2), np.uint8)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)

        white_contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Combine the two masks
        combined_mask = cv2.bitwise_or(white_mask, green_mask)
            
        return white_contours, green_contours, combined_mask
    
    def calculate_distance(self, contour, known_width):
        x, y, w, h = cv2.boundingRect(contour)
        distance_2d = (known_width * self.focal_length) / w
        return distance_2d, (x + w // 2, y + h // 2)

    def calculate_3d_distance(self, distance_2d):
        distance_3d = np.sqrt(distance_2d**2 + self.camera_height**2)
        return distance_3d

    def calculate_object_position(self, distance, center, image_center):
        dx = center[0] - image_center[0]
        y = (dx * distance)  / self.focal_length  # y 값 계산, 왼쪽이 음수, 오른쪽이 양수
        x = distance  # x 값은 distance_2d
        return x, -y

    def edge_enhance(self, frame):
        kernel = np.array([[0, -1, 0],
                           [-1, 5, -1],
                           [0, -1, 0]])
        enhanced_frame = cv2.filter2D(frame, -1, kernel)
        return enhanced_frame

    def simple_road_masking(self, frame):
        h, w, _ = frame.shape
        polygon_points = np.array([
            [0, h],                # Bottom-left corner
            [0, h*0.4],        # Mid-left point
            [w*0.45, h*0.05],        # Top-left point
            [w*0.55, h*0.05],        # Top-right point
            [w, h*0.4],        # Mid-right point
            [w, h]                 # Bottom-right corner
        ], np.int32)
    
        polygon_points = polygon_points.reshape((-1, 1, 2))
        mask = np.zeros_like(frame)
        cv2.fillPoly(mask, [polygon_points], (255, 255, 255))
        masked_frame = cv2.bitwise_and(frame, mask)
        return masked_frame

    def road_masking(self, frame):
        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        edges = cv2.Canny(blur, 50, 150)
        
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=50)
        
        current_left_lines = []
        current_right_lines = []
        
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else 0
                if slope < 0:  # Left lane
                    current_left_lines.append(line)
                elif slope > 0:  # Right lane
                    current_right_lines.append(line)
        
        def average_line(lines):
            if len(lines) == 0:
                return None
            x_coords = []
            y_coords = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                x_coords += [x1, x2]
                y_coords += [y1, y2]
            poly = np.polyfit(x_coords, y_coords, 1)
            slope, intercept = poly
            y1 = frame.shape[0]
            y2 = int(y1 * 0.05)
            x1 = int((y1 - intercept) / slope)
            x2 = int((y2 - intercept) / slope)
            return np.array([x1, y1, x2, y2], dtype=np.int32)
        
        left_line = average_line(current_left_lines)
        right_line = average_line(current_right_lines)

        # Add the current frame's lines to the history
        if left_line is not None:
            self.left_lines.append(left_line)
            if len(self.left_lines) > self.num_frames:
                self.left_lines.pop(0)

        if right_line is not None:
            self.right_lines.append(right_line)
            if len(self.right_lines) > self.num_frames:
                self.right_lines.pop(0)

        # Calculate the moving average of the lines
        def moving_average(lines):
            if len(lines) == 0:
                return None
            avg_line = np.mean(lines, axis=0)
            return avg_line.astype(int)

        left_line_avg = moving_average(self.left_lines)
        right_line_avg = moving_average(self.right_lines)

        # Get the height of the frame
        h, w = frame.shape[:2]

        # Ensure the lines do not cross by checking their x-coordinates at the same y level
        if left_line_avg is not None and right_line_avg is not None:
            # Check for crossing and adjust x-coordinates if necessary
            if left_line_avg[2] > right_line_avg[2]:  # If left line x2 is greater than right line x2
                mid_y = (left_line_avg[3] + right_line_avg[3]) // 2
                left_x_at_mid = int((mid_y - left_line_avg[3]) / (left_line_avg[1] - left_line_avg[3]) * (left_line_avg[0] - left_line_avg[2]) + left_line_avg[2])
                right_x_at_mid = int((mid_y - right_line_avg[3]) / (right_line_avg[1] - right_line_avg[3]) * (right_line_avg[0] - right_line_avg[2]) + right_line_avg[2])
                if left_x_at_mid > right_x_at_mid:
                    left_line_avg[2] = w*0.45
                    right_line_avg[2] = w*0.55  # Ensure a minimum gap between the lines
            if left_line_avg[0] > right_line_avg[0]:
                left_line_avg[0] = 0
                right_line_avg[0] = w

        
        
        # Define polygon points for the mask
        if left_line_avg is not None and right_line_avg is not None:
            polygon_points = np.array([
                [left_line_avg[0], left_line_avg[1]],
                [left_line_avg[2], left_line_avg[3]],
                [right_line_avg[2], right_line_avg[3]],
                [right_line_avg[0], right_line_avg[1]]
            ], np.int32)
        else:
            # If lines are not detected, default to a general area
            polygon_points = np.array([
                [0, h],                # Bottom-left corner
                [0, h*0.4],            # Mid-left point
                [w*0.45, h*0.05],      # Top-left point
                [w*0.55, h*0.05],      # Top-right point
                [w, h*0.4],            # Mid-right point
                [w, h]                 # Bottom-right corner
            ], np.int32)
        
        # Reshape the array to fit the expected input format for cv2.fillPoly
        polygon_points = polygon_points.reshape((-1, 1, 2))
        
        # Create a mask with the same dimensions as the frame, initialized to zero (black)
        mask = np.zeros_like(frame)
        
        # Fill the polygon on the mask with white (255, 255, 255)
        cv2.fillPoly(mask, [polygon_points], (255, 255, 255))
        
        # Apply the mask to the frame using bitwise_and
        masked_frame = cv2.bitwise_and(frame, mask)
        
        return masked_frame



    def execute(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.RH.frame is None:
                continue
            frame_copy = deepcopy(self.RH.frame)
            masked_frame = self.road_masking(frame_copy)
            hsv = cv2.cvtColor(masked_frame, cv2.COLOR_BGR2HSV)
            hsv_sharpe = self.edge_enhance(hsv)
            white_contours, green_contours, combined_mask = self.detect_boxes(hsv_sharpe)
            h, w, _ = frame_copy.shape
            image_center = (w // 2, h // 2)
            
            # masked_frame = cv2.bitwise_and(hsv_sharpe, hsv_sharpe, mask=combined_mask)
            # brighter_frame = cv2.addWeighted(masked_frame, 1.5, np.zeros_like(masked_frame), 0, 0)
            # result_frame = cv2.bitwise_or(hsv_sharpe, brighter_frame, mask=combined_mask)

            result_frame = frame_copy

            positions = []
            for contour in white_contours:
                c_area = cv2.contourArea(contour)
                if 80 < c_area < 10000:
                    distance_2d, center = self.calculate_distance(contour, self.white_box_width)
                    distance_3d = self.calculate_3d_distance(distance_2d)
                    object_x, object_y = self.calculate_object_position(distance_2d, center, image_center)
                    positions.append((object_x, object_y))
                    
                    cv2.putText(result_frame, f"White x:{object_x} y:{object_y}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.drawContours(result_frame, [contour], -1, (255, 255, 255), 2)

            for contour in green_contours:
                c_area = cv2.contourArea(contour)
                if 80 < c_area < 10000:
                    distance_2d, center = self.calculate_distance(contour, self.green_box_width)
                    distance_3d = self.calculate_3d_distance(distance_2d)
                    object_x, object_y = self.calculate_object_position(distance_2d, center, image_center)
                    positions.append((object_x, object_y))
                    
                    cv2.putText(result_frame, f"Green x:{object_x} y:{object_y}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.drawContours(result_frame, [contour], -1, (0,255,0), 2)

            self.RH.publish(result_frame, positions)
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
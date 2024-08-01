#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from PIL import Image
import io

import os

output_folder = "/home/inha/Documents/bag/20240801/data/"

if not os.path.exists(output_folder):
    os.makedirs(output_folder)

def get_next_image_number(folder):
    # 폴더 내 파일 목록을 가져와서 정렬
    files = sorted(os.listdir(folder))
    max_num = 0

    # 파일이름이 6자리 숫자이면서 .png인 경우 가장 큰 숫자 찾기
    for file in files:
        if file.endswith(".png") and file[:-4].isdigit():
            num = int(file[:-4])
            if num > max_num:
                max_num = num

    return max_num + 1

class ImageSaver:
    def __init__(self):
        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/stereo/left/image_raw/compressed", CompressedImage, self.image_callback)
        self.image_sub = rospy.Subscriber("/camera/image_color/compressed", CompressedImage, self.image_callback)
        #self.image_sub = rospy.Subscriber("/stereo/left/image_raw", Image, self.image_callback)
        self.image = None
        self.image_count = get_next_image_number(output_folder)
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)  # 1초마다 호출

    def image_callback(self, data):
        #self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            # Handle different encodings
            if data.format == "bayer_rggb8; jpeg compressed ":
                # Debayer the image
                self.image = cv2.cvtColor(cv_image, cv2.COLOR_BayerBG2BGR)
            elif data.format == "8UC3":
                pass  # Already in RGB format
            elif data.format == "8UC1":
                self.image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
            elif data.format == "16UC1" or data.format == "32FC1":
                min_val = 0
                max_val = 255
                self.image = cv2.normalize(cv_image, None, min_val, max_val, cv2.NORM_MINMAX)
                self.image = cv2.convertScaleAbs(self.image)
                self.image = cv2.cvtColor(self.image, cv2.COLOR_GRAY2RGB)
            else:
                self.image = cv_image
        except CvBridgeError as e2:
            rospy.logerr(f"Failed to convert image: {e2}")
            return
        
    def timer_callback(self, event):
        if self.image is not None:
            try:
                file_name = os.path.join(output_folder, f"{self.image_count:06}.png")
                cv2.imwrite(file_name, self.image)
                rospy.loginfo(f"Saved image {file_name}")
                self.image_count += 1
            except Exception as e:
                rospy.logerr(f"Failed to save image: {e}")

if __name__ == '__main__':
    try:
        rospy.init_node('image_saver', anonymous=True)
        image_saver = ImageSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

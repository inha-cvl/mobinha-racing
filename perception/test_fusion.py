# import time
# for i in range(1,100):
#     print("hello")
#     time.sleep(0.5)
import math
import numpy as np
from scipy.optimize import linear_sum_assignment
from shapely.geometry import Polygon



def rotate_point(px, py, cx, cy, angle):

    sin_theta = math.sin(angle)
    cos_theta = math.cos(angle)
    
    translated_x = px - cx
    translated_y = py - cy
    
    rotated_x = translated_x * cos_theta - translated_y * sin_theta + cx
    rotated_y = translated_x * sin_theta + translated_y * cos_theta + cy
    
    return rotated_x, rotated_y



def calculate_iou(obj1, obj2, size_x, size_y):
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

    rotated_corners1 = [rotate_point(px, py, x1, y1, heading1) for (px, py) in corners1]
    rotated_corners2 = [rotate_point(px, py, x2, y2, heading2) for (px, py) in corners2]

    poly1 = Polygon(rotated_corners1)
    poly2 = Polygon(rotated_corners2)

    inter_area = poly1.intersection(poly2).area
    
    box1_area = poly1.area
    box2_area = poly2.area
    
    iou = inter_area / (box1_area + box2_area - inter_area)
    return iou

def lidar_radar_matching(lidar_objects, radar_objects,iou_threshold = 0.1, size_x = 4.365, size_y = 1.85):
    iou_matrix = np.zeros((len(lidar_objects), len(radar_objects)))

    for i, lidar_obj in enumerate(lidar_objects):
        for j, radar_obj in enumerate(radar_objects):
            iou = calculate_iou(lidar_obj, radar_obj, size_x, size_y)
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


import matplotlib.pyplot as plt
import random

def generate_test_objects(num_objects, x_range, y_range):
    objects = []
    for i in range(num_objects):
        x = random.uniform(*x_range)  # x 좌표 범위
        y = random.uniform(*y_range)  # y 좌표 범위
        obs_3 = random.uniform(0, 1)  # 임의의 obs[3] 값
        heading = random.uniform(0, 2 * math.pi)  # 0~360도에서 임의의 heading 값 (라디안)
        distance = random.uniform(1, 100)  # 임의의 거리 값
        objects.append([i, x, y, obs_3, heading, distance])
    return objects
import matplotlib.patches as patches
def plot_objects_with_boxes(lidar_objects, radar_objects, matched_pairs, size_x=4.365, size_y=1.85):
    plt.figure(figsize=(10, 8))

    # LiDAR objects (파란색)
    for obj in lidar_objects:
        plt.plot(obj[1], obj[2], 'bo', label='LiDAR Object' if obj == lidar_objects[0] else "")
        plt.text(obj[1], obj[2], f'L{obj[0]}', color='blue')

        # LiDAR object 주변 박스 그리기
        corners = [
            (obj[1] - size_x / 2, obj[2] - size_y / 2),
            (obj[1] + size_x / 2, obj[2] - size_y / 2),
            (obj[1] + size_x / 2, obj[2] + size_y / 2),
            (obj[1] - size_x / 2, obj[2] + size_y / 2)
        ]
        rotated_corners = [rotate_point(px, py, obj[1], obj[2], obj[4]) for (px, py) in corners]
        poly = Polygon(rotated_corners)
        x, y = poly.exterior.xy
        plt.plot(x, y, 'b-')

    # Radar objects (빨간색)
    for obj in radar_objects:
        plt.plot(obj[1], obj[2], 'ro', label='Radar Object' if obj == radar_objects[0] else "")
        plt.text(obj[1], obj[2], f'R{obj[0]}', color='red')

        # Radar object 주변 박스 그리기
        corners = [
            (obj[1] - size_x / 2, obj[2] - size_y / 2),
            (obj[1] + size_x / 2, obj[2] - size_y / 2),
            (obj[1] + size_x / 2, obj[2] + size_y / 2),
            (obj[1] - size_x / 2, obj[2] + size_y / 2)
        ]
        rotated_corners = [rotate_point(px, py, obj[1], obj[2], obj[4]) for (px, py) in corners]
        poly = Polygon(rotated_corners)
        x, y = poly.exterior.xy
        plt.plot(x, y, 'r-')

    # 매칭된 객체 선으로 연결
    # for lidar_idx, radar_idx in matched_pairs:
    #     lidar_obj = lidar_objects[lidar_idx]
    #     radar_obj = radar_objects[radar_idx]
    #     plt.plot([lidar_obj[1], radar_obj[1]], [lidar_obj[2], radar_obj[2]], 'g--')

    plt.legend()
    plt.title('LiDAR-Radar Object Matching with Bounding Boxes')
    plt.xlabel('X (ENU 좌표계)')
    plt.ylabel('Y (ENU 좌표계)')
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    # LiDAR와 Radar 객체를 생성 (각각 6개, 4개)
    lidar_objects = generate_test_objects(6, (-8, 8), (-8, 8))
    radar_objects = generate_test_objects(4, (-8, 8), (-8, 8))

    # LiDAR와 Radar 객체 매칭
    lidar_objects, matched_pairs = lidar_radar_matching(lidar_objects, radar_objects)

    # 매칭 결과 시각화 (박스 포함)
    plot_objects_with_boxes(lidar_objects, radar_objects, matched_pairs)

    # 매칭된 객체 정보 출력
    print("Matched LiDAR and Radar objects:")
    for lidar_idx, radar_idx in matched_pairs:
        print(f"LiDAR Object {lidar_idx} matched with Radar Object {radar_idx}")
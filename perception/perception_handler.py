import math
import numpy as np
from scipy.optimize import linear_sum_assignment

def distance( x1, y1, x2, y2):
    return np.sqrt((x2-x1)**2+(y2-y1)**2)

def calculate_radar_heading_velocity(rel_pos_x, rel_pos_y, rel_vel_x, rel_vel_y):
        heading_radians = math.atan2(rel_vel_x, -rel_vel_y)
        heading_degrees = (math.degrees(heading_radians)+90)
        relative_speed = ((rel_vel_x*rel_pos_x)+(rel_vel_y*rel_pos_y))/math.sqrt(rel_pos_x**2 + rel_pos_y**2)
        return heading_degrees, relative_speed

def cluster_radar_obstacles( data, distance_threshold=0.5):
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

            if distance(x1, y1, x2, y2) <= distance_threshold:
                cluster.append(data[j])
                visited[j] = True
        
        if len(cluster) > 1:
            max_age_point = max(cluster, key=lambda x: x[4])
            x = 0
            y = 0
            for c in cluster:
                x += c[0]
                y += c[1]
            x = x/len(cluster)
            y = y/len(cluster)

            heading,velocity = calculate_radar_heading_velocity(x1, y1, max_age_point[2], max_age_point[3])
            clusters.append([x1, y1,  heading, velocity, float(max_age_point[4])])

        else:
            heading,velocity = calculate_radar_heading_velocity(x1, y1, data[i][2], data[i][3])
            clusters.append([data[i][0], data[i][1], heading, velocity, float(data[i][4])])  
    return clusters

def filtering_by_spd(data, spd):
    filtered_list = []
    for obj in data:
        if obj[3] - spd == 0:
            pass
        else:
            filtered_list.append(obj)
    return filtered_list




def project_to_image(pts_3d, P, intrinsic):
    pts_3d_hom = np.vstack((pts_3d, np.ones((1, pts_3d.shape[1]))))
    pts_2d_hom = P.dot(pts_3d_hom)
    pts_2d_img = intrinsic.dot(pts_2d_hom)
    pts_2d = pts_2d_img[:2, :] / pts_2d_img[2,:]
    return pts_2d


def rectify_corners(objects_3d, P, intrinsic):
    object_2d = []
    
    for i , object_3d in enumerate(objects_3d):
        projected_corners = project_to_image(object_3d[0].T, P, intrinsic)
        pos_x, pos_y, pos_z = object_3d[1][:]
        x_min = int(np.min(projected_corners[0, :]))
        x_max = int(np.max(projected_corners[0, :]))
        y_min = int(np.min(projected_corners[1, :]))
        y_max = int(np.max(projected_corners[1, :]))
        
        object_2d.append((x_min, y_min,x_max, y_max, pos_x, pos_y, pos_z))
    return object_2d


def find_corners(clustered_list): 
    corners_list = []
    # # for car
    # center_z = 0.4
    # size_x = 4.65
    # size_y = 1.82
    # size_z = 1.4
    
    # human
    center_z = 0.55
    size_x = 0.3
    size_y = 0.9
    size_z = 1.8
    

    for (center_x,center_y, _, _, _) in clustered_list:
        corners = np.array([[(center_x+0.8) + size_x/2, center_y + size_y/2, center_z + size_z/2],
                            [(center_x+0.8) + size_x/2, center_y + size_y/2, center_z - size_z/2],
                            [(center_x+0.8) + size_x/2, center_y - size_y/2, center_z + size_z/2],
                            [(center_x+0.8) + size_x/2, center_y - size_y/2, center_z - size_z/2],
                            [(center_x+0.8) - size_x/2, center_y + size_y/2, center_z + size_z/2],
                            [(center_x+0.8) - size_x/2, center_y + size_y/2, center_z - size_z/2],
                            [(center_x+0.8) - size_x/2, center_y - size_y/2, center_z + size_z/2],
                            [(center_x+0.8) - size_x/2, center_y - size_y/2, center_z - size_z/2]])
        position = np.array([(center_x+0.8) , center_y, center_z]).T
        corners_list.append([corners, position])

    return corners_list



def compute_iou(box1, box2):
    x1_min, y1_min, x1_max, y1_max, *rest1 = box1
    x2_min, y2_min, x2_max, y2_max, *rest2 = box2

    inter_x_min = max(x1_min, x2_min)
    inter_y_min = max(y1_min, y2_min)
    inter_x_max = min(x1_max, x2_max)
    inter_y_max = min(y1_max, y2_max)

    inter_area = max(0, inter_x_max - inter_x_min + 1) * max(0, inter_y_max - inter_y_min + 1)

    box1_area = (x1_max - x1_min + 1) * (y1_max - y1_min + 1)
    box2_area = (x2_max - x2_min + 1) * (y2_max - y2_min + 1)

    iou = inter_area / float(box1_area + box2_area - inter_area)
    return iou


def create_iou_matrix(points_2d, bounding_boxes):
    iou_matrix = np.zeros((len(points_2d), len(bounding_boxes)))

    for i, point_box in enumerate(points_2d):
        for j, detection_box in enumerate(bounding_boxes):
            iou_matrix[i, j] = compute_iou(point_box, detection_box)

    return iou_matrix


def match_bounding_boxes(points_2d, bounding_boxes, iou_threshold=0.3):
    iou_matrix = create_iou_matrix(points_2d, bounding_boxes)

    row_indices, col_indices = linear_sum_assignment(-iou_matrix) 
    
    matched_boxes = []
    for i, j in zip(row_indices, col_indices):
        if iou_matrix[i, j] >= iou_threshold: 
            matched_boxes.append([bounding_boxes[j], points_2d[i][4:]])

    return matched_boxes

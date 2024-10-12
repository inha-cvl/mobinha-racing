import numpy as np
import copy
import heapq as hq

import rospy
from math import *
from scipy.ndimage import gaussian_filter1d

import global_path

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

lanelets = None
tiles = None
tile_size = None
graph = None
lane_width = None

def euc_distance(pt1, pt2):
    return np.sqrt((pt2[0]-pt1[0])**2+(pt2[1]-pt1[1])**2)

def find_nearest_idx(pts, pt):
    min_dist = float('inf')
    min_idx = 0

    for idx, pt1 in enumerate(pts):
        dist = euc_distance(pt1, pt)
        if dist < min_dist:
            min_dist = dist
            min_idx = idx

    return min_idx

def lanelet_matching(t_pt):
    row = int(t_pt[0] // tile_size)
    col = int(t_pt[1] // tile_size)

    min_dist = float('inf')
    l_id, l_idx = None, None

    for i in range(-1, 2):
        for j in range(-1, 2):
            selected_tile = tiles.get((row+i, col+j))
            if selected_tile is not None:
                for id_, data in selected_tile.items():
                    for idx, pt in enumerate(data['waypoints']):
                        dist = euc_distance(t_pt, pt)
                        if dist < min_dist:
                            min_dist = dist
                            l_id = id_
                            l_idx = data['idx'][idx]
    if l_id is not None:
        return (l_id, l_idx)
    else:
        return None


def convert_kmh_to_ms(speed_kmh):
    return speed_kmh / 3.6

def get_straight_path(idnidx, path_len, stop_id, prior='Left'):
    s_n = idnidx[0]
    s_i = idnidx[1]
    wps = copy.deepcopy(lanelets[s_n]['waypoints'])
    lls_len = len(wps)
    ids = [s_n]*lls_len
    vs = [lanelets[s_n]['speedLimit']]*lls_len
    u_n = s_n
    u_i = s_i+int(path_len)#*M_TO_IDX)
    e_i = u_i
        
    while u_i >= lls_len:
        
        _u_n = get_possible_successor(u_n, prior)
        if _u_n == stop_id:
            u_i = lls_len-1
            break
        if _u_n == None:
            e_i = len(wps)
            break
        u_n = _u_n
        u_i -= lls_len
        e_i += u_i
        u_wp = lanelets[u_n]['waypoints']
        lls_len = len(u_wp)
        ids.extend([u_n]*lls_len)
        v = lanelets[u_n]['speedLimit']
        vs.extend([v]*lls_len)
        wps += u_wp

    r = wps[s_i:e_i]
    path_ids = ids[s_i:e_i]
    path_vs = vs[s_i:e_i]

    return r, [u_n, u_i], path_ids, path_vs

def get_cut_idx_ids(id):
    idx_list = lanelets[id]['cut_idx']
    llen = idx_list[-1][-1]
    ids = [None]*llen
    for i in range(len(ids)):
        for j, (minv, maxv) in enumerate(idx_list):
            if minv <= i < maxv:
                ids[i]=id+'_'+str(j)
                break
    return ids

def get_merged_point(idnidx, path_len, to=1):
        wps, [u_n, u_i],_,_ = get_straight_path(idnidx, path_len, '')
        c_pt = wps[-1]
        l_id, r_id = get_neighbor( u_n)
        n_id = l_id if to == 1 else r_id
        if n_id != None:
            r = lanelets[n_id]['waypoints']
            u_n = n_id
            u_i = find_nearest_idx(r, c_pt)

        return [u_n, u_i]
    

def get_possible_successor(node, prior='Left'):
    successor = None
    left_lanes, right_lanes, me = get_whole_neighbor(node)
    if len(lanelets[node]['successor']) <= 0:
        if prior == 'Left':
            check_a = left_lanes
            check_b = right_lanes
        else:   
            check_a = right_lanes
            check_b = left_lanes
        
        most_successor = find_most_successor(check_a)
        if most_successor == None:
            most_successor = find_most_successor(check_b)

        successor = most_successor
    else:
        if prior == 'Left':
            i = 0
        else:
            i = -1
        successor = lanelets[node]['successor'][i]

    return successor

# hees solchan global path
def get_pocket_successor(node, prior='Left'):
    successor = get_possible_successor(node, prior)
    successor_idx = None

    if successor is not None:
        current_waypoints = lanelets[node]['waypoints']
        target_point = current_waypoints[-1] if current_waypoints else (0, 0)
        
        successor_waypoints = lanelets[successor]['waypoints']
        successor_idx = find_nearest_idx(successor_waypoints, target_point)

    return [successor, successor_idx]

def get_whole_neighbor(node):
    num = 1
    find_node = node
    left_most = True
    right_most = True
    left_lanes = []
    right_lanes = []

    while left_most:
        if lanelets[find_node]['adjacentLeft'] != None:
            find_node = lanelets[find_node]['adjacentLeft']
            left_lanes.append(find_node)
            num += 1
        else:
            left_most = False
            find_node = node
    
    while right_most:
        if lanelets[find_node]['adjacentRight'] != None:
            find_node = lanelets[find_node]['adjacentRight']
            right_lanes.append(find_node)
            num += 1
            
        else:
            right_most = False
            find_node = node

    me_idx = len(left_lanes)

    return left_lanes, right_lanes, me_idx

def find_most_successor(check_l):
    most_successor = None
    for c in check_l:
        if len(lanelets[c]['successor']) <= 0:
            continue
        else:
            most_successor = lanelets[c]['successor'][0]
            break
    return most_successor


def node_matching(idnidx):
    node_id = idnidx[0]
    if lanelets[idnidx[0]].get('cut_idx') is not None:
        for n, (s_idx, e_idx) in enumerate(lanelets[idnidx[0]]['cut_idx']):
            if idnidx[1] >= s_idx and idnidx[1] < e_idx:
                node_id += '_%s' % (n)
                break

    return node_id

def dijkstra(start, finish):
    distances = {}
    previous = {}
    nodes = []

    for vertex in graph:
        if vertex == start:
            distances[vertex] = 0
            hq.heappush(nodes, [distances[vertex], vertex])
        else:
            distances[vertex] = float('inf')
            hq.heappush(nodes, [distances[vertex], vertex])
        previous[vertex] = None

    while nodes:
        current = hq.heappop(nodes)[1]

        if current == finish:
            path = []
            if previous[current] is not None:
                while previous[current]:
                    path.append(current)
                    current = previous[current]
                path.append(start)
                path.reverse()
                cost = distances[finish]
                return (path, cost)

            else:
                return None

        neighbors = graph[current]

        for neighbor in neighbors:
            if neighbor == start:
                continue
            # cost(start->current) + cost(current->neighbor)
            bridge_cost = distances[current] + neighbors[neighbor]

            # found shortest path! -> update!
            if bridge_cost < distances[neighbor]:
                distances[neighbor] = bridge_cost
                previous[neighbor] = current

                for node in nodes:
                    if node[1] == neighbor:
                        node[0] = bridge_cost
                        break
                hq.heapify(nodes)  # heapq relocate

    return None




def cut_by_start_goal(sidnidx, gidnidx, path_from_id):
    waypoints = path_from_id[0]
    s_idx = min(range(len(waypoints)), key=lambda i: euc_distance(waypoints[i], sidnidx[1]))
    g_idx = min(range(len(waypoints)), key=lambda i: euc_distance(waypoints[i], gidnidx[1]))
    return path_from_id[0][s_idx:g_idx+1],path_from_id[1][s_idx:g_idx+1],path_from_id[2][s_idx:g_idx+1]

def get_neighbor(node):
    l_id = lanelets[node]['adjacentLeft']
    r_id = lanelets[node]['adjacentRight']
    return l_id, r_id

def gaussian_smoothing_2d(points, sigma=1):
    wx, wy = zip(*points)
    smoothed_wx = gaussian_filter1d(wx, sigma=sigma)
    smoothed_wy = gaussian_filter1d(wy, sigma=sigma)
    return list(zip(smoothed_wx, smoothed_wy))

def smooth_interpolate(points, precision):
    points = filter_same_points(points)
    smoothed_path = gaussian_smoothing_2d(points)
    wx, wy = zip(*smoothed_path)
    if len(wx) > 1 and len(wy) > 1:
        itp = global_path.libs.quadratic_spline_interpolate.QuadraticSplineInterpolate(list(wx), list(wy))
        itp_points = []
        for ds in np.arange(0.0, itp.s[-1], precision):
            x, y = itp.calc_position(ds)
            itp_points.append((float(x), float(y)))

        return itp_points
    else:
        return []

def filter_same_points(points):
    filtered_points = []
    pre_pt = None

    for pt in points:
        if pre_pt is None or pt != pre_pt:
            filtered_points.append(pt)

        pre_pt = pt

    return filtered_points


def node_to_waypoints(shortest_path, sidnidx, gidnidx):
    final_path = []
    final_ids = []
    final_vs = []

    for i, id in enumerate(shortest_path):
        alpha_path = []
        v = 0
        _id = id

        split_id = id.split('_')
        
        if len(split_id) == 2:
            _id = split_id[0]
            s_idx, e_idx = (lanelets[_id]['cut_idx'][int(split_id[1])])
            alpha_path = lanelets[_id]['waypoints'][s_idx:e_idx]
        else:
            _id = id
            alpha_path = lanelets[_id]['waypoints']

        v = lanelets[_id]['speedLimit']
        lls_len = len(alpha_path)
        final_vs.extend([v]*lls_len)
        final_ids.extend([_id]*lls_len)
        final_path.extend(alpha_path)
       
    return final_path, final_ids, final_vs

def calc_norm_vec(points):
    theta = atan2(points[1][1]-points[0][1], points[1][0]-points[0][0])
    A = sin(theta)
    B = -cos(theta)
    return A,B,theta



def calc_kappa(epoints, npoints):
    if abs(npoints[1][1]-epoints[1]) == 0 or abs(npoints[1][0]-epoints[0]) == 0:
        Rk = 0
    elif abs(npoints[0][1]-epoints[1]) == 0 or abs(npoints[0][0]-epoints[0]) == 0:
        Rk = 0
    else:
        if abs(npoints[1][1]-epoints[1]) < abs(npoints[1][0]-epoints[0]):
            dydx2 = (npoints[1][1]-epoints[1])/(npoints[1][0]-epoints[0])
            dydx1 = (npoints[0][1]-epoints[1])/(npoints[0][0]-epoints[0])
            dydx = (dydx2+dydx1)/2
            d2ydx2 = 2*(dydx2-dydx1)/(npoints[1][0]-npoints[0][0])
            Rk = d2ydx2/((1+(dydx)**2)**(3/2))
        else:
            dxdy2 = (npoints[1][0]-epoints[0])/(npoints[1][1]-epoints[1])
            dxdy1 =(npoints[0][0]-epoints[0])/(npoints[0][1]-epoints[1])
            dxdy = (dxdy2+dxdy1)/2
            d2xdy2 = 2*(dxdy2-dxdy1)/(npoints[1][1]-npoints[0][1])
            Rk = d2xdy2/((1+(dxdy)**2)**(3/2))
    return Rk

from scipy.interpolate import CubicSpline

import numpy as np
from scipy.interpolate import CubicSpline

def calc_kappa_spline(epoints, npoints, t=0.5):
    """
    3차 스플라인을 사용하여 곡률을 계산하는 함수.
    
    :param epoints: 현재 포인트 (x, y 좌표)
    :param npoints: 이전 포인트와 이후 포인트 (2개의 [x, y] 좌표 리스트)
    :param t: 현재 포인트에서의 상대적인 위치 값 (0 <= t <= 1 범위)
    
    :return: 현재 포인트에서의 곡률 값 Rk
    """
    
    # epoints와 npoints에서 x와 y 좌표 값을 가져옴
    x_vals = [npoints[0][0], epoints[0], npoints[1][0]]  # 이전, 현재, 이후의 x 좌표들
    y_vals = [npoints[0][1], epoints[1], npoints[1][1]]  # 이전, 현재, 이후의 y 좌표들
    
    # 3차 스플라인을 x와 y 축에 대해 각각 적합
    spline_x = CubicSpline([0, 1, 2], x_vals)  # t = 1이 현재 포인트
    spline_y = CubicSpline([0, 1, 2], y_vals)

    # 스플라인의 1차, 2차 미분값 계산
    x_prime = spline_x(t, 1)  # x의 1차 미분
    y_prime = spline_y(t, 1)  # y의 1차 미분
    x_double_prime = spline_x(t, 2)  # x의 2차 미분
    y_double_prime = spline_y(t, 2)  # y의 2차 미분

    # 곡률 계산 공식
    numerator = x_prime * y_double_prime - y_prime * x_double_prime
    denominator = (x_prime**2 + y_prime**2)**(3/2)

    if denominator == 0:
        return 0.0  # 곡률이 정의되지 않는 경우
    else:
        Rk = numerator / denominator
        return Rk



def compute_curvature_radius(path, tg_idx=50):
    curvature_radii = []
    path_len = len(path)
    
    for i in range(path_len):
        dynamic_idx = min(i, tg_idx, path_len - i - 1)

        x1, y1 = path[i - dynamic_idx]
        x2, y2 = path[i]
        x3, y3 = path[i + dynamic_idx]
        
        dx1 = x2 - x1
        dy1 = y2 - y1
        dx2 = x3 - x2
        dy2 = y3 - y2
        
        ddx = dx2 - dx1
        ddy = dy2 - dy1
        
        numerator = abs(dx1 * ddy - dy1 * ddx)
        denominator = (dx1**2 + dy1**2)**1.5
        
        if denominator != 0:
            curvature = numerator / denominator
        else:
            curvature = 1e-3 
        
        if numerator == 0:
            curvature = 100
        curvature_radii.append(curvature)

    window_size = tg_idx
    smoothed_radii = np.convolve(curvature_radii, np.ones(window_size)/window_size, mode='same')
    smoothed_radii2 = np.convolve(smoothed_radii, np.ones(window_size)/window_size, mode='same')
    return smoothed_radii2

def get_profiles(vs, sec_to_reach):
    total_time = int((len(vs) / 80))
    peak_time = sec_to_reach/2
    accel_time = sec_to_reach
    t = np.linspace(0, total_time, total_time*10)

    # 가속도 프로파일 생성
    acceleration = np.zeros_like(t)  # 가속도 초기화

    # 가속 구간 설정
    acceleration[t <= peak_time] = (t[t <= peak_time] / peak_time)
    acceleration[(t > peak_time) & (t < accel_time)] = (1- (t[(t > peak_time) & (t < accel_time)] - peak_time) / peak_time) 

    # 감속 구간 설정
    decel_start = total_time - accel_time
    decel_peak = total_time - accel_time + peak_time
    acceleration[(t >= decel_start) & (t <= decel_peak)] = -((t[(t >= decel_start) & (t <= decel_peak)] - decel_start) / peak_time)
    acceleration[(t > decel_peak) & (t < total_time)] = -1+((t[(t > decel_peak) & (t < total_time)] - decel_peak) / peak_time) 

    acceleration = acceleration * (max_vel/peak_time)

    # 속도 프로파일 계산 (가속도 적분)
    velocity = np.cumsum(acceleration) * (t[1] - t[0])
    # 거리 프로파일 계산 (속도 적분)
    distance = np.cumsum(velocity) * (t[1] - t[0])

    return acceleration, velocity, distance

def convert_kmh_to_ms(speed_kmh):
    return speed_kmh / 3.6

def adjust_velocity_profile(velocity_profile):
    accel_distance = 15
    decel_distance = 30
    init_accel_len = 15
    final_decel_len = 30
    time_interval = 1

    initial_acceleration_profile = np.linspace(0, velocity_profile[0], init_accel_len).tolist()
    
    final_deceleration_profile = np.linspace(velocity_profile[-1], 0, final_decel_len).tolist()
    
    # adjusted_profile = initial_acceleration_profile
    # i = init_accel_len

    # while i < len(velocity_profile) - final_decel_len:
    #     if velocity_profile[i] != velocity_profile[i-1]:
    #         if velocity_profile[i] < velocity_profile[i-1]:
    #             transition_start = max(i - decel_distance, init_accel_len)
    #             transition = np.linspace(velocity_profile[i-1], velocity_profile[i], decel_distance).tolist()
    #             adjusted_profile = adjusted_profile[:transition_start] + transition + [velocity_profile[i]]
    #             i += 1
    #         else:
    #             transition = np.linspace(velocity_profile[i-1], velocity_profile[i], accel_distance).tolist()
    #             adjusted_profile.extend(transition[1:])
    #             i += accel_distance - 1
    #     else:
    #         adjusted_profile.append(velocity_profile[i])
    #         i += 1

    # adjusted_profile.extend(final_deceleration_profile)
    adjusted_profile = velocity_profile

    # Calculate acceleration profile
    acceleration_profile = [0] * len(adjusted_profile)
    
    # Initial acceleration
    for i in range(1, init_accel_len):
        acceleration_profile[i] = 2.5 * i / init_accel_len
    
    # Final deceleration
    for i in range(len(adjusted_profile) - final_decel_len, len(adjusted_profile)):
        acceleration_profile[i] = 2 * (i - (len(adjusted_profile) - final_decel_len)) / final_decel_len
    
    # Middle part
    i = init_accel_len
    while i < len(adjusted_profile) - final_decel_len:
        if adjusted_profile[i] != adjusted_profile[i-1]:
            if adjusted_profile[i] > adjusted_profile[i-1]:
                for j in range(accel_distance):
                    if i + j < len(acceleration_profile):
                        acceleration_profile[i + j] = 2.5 * (j + 1) / accel_distance
                for j in range(accel_distance):
                    if i + accel_distance + j < len(acceleration_profile):
                        acceleration_profile[i + accel_distance + j] = 2.5 * (accel_distance - j - 1) / accel_distance
            elif adjusted_profile[i] < adjusted_profile[i-1]:
                for j in range(decel_distance):
                    if i + j < len(acceleration_profile):
                        acceleration_profile[i + j] = 2 * (j + 1) / decel_distance
                for j in range(decel_distance):
                    if i + decel_distance + j < len(acceleration_profile):
                        acceleration_profile[i + decel_distance + j] = 2 * (decel_distance - j - 1) / decel_distance
            i += 2 * decel_distance
        else:
            i += 1
    
    distance = [0]

    for i in range(1, len(adjusted_profile)):
        distance_traveled = adjusted_profile[i-1] * time_interval + 0.5 * acceleration_profile[i-1] * (time_interval ** 2)
        distance.append(distance[-1] + distance_traveled)
    
    distance = np.array(distance)

    adjusted_profile = [convert_kmh_to_ms(speed) for speed in adjusted_profile]

    return adjusted_profile, acceleration_profile, distance



def get_lane_width(id):
    h_w = (lane_width / 2)-0.5
    l_n, r_n = get_neighbor(id)
    if l_n != None or r_n != None:
        lane_num = lanelets[id]['laneNo']
        if id in ['47', '10']:
            l_w = h_w
            r_w = lane_width+h_w
        elif id in ['68', '28', '29']:
            l_w = r_w = h_w
        if lane_num == 1:
            l_w = h_w
            r_w = (2*lane_width)+h_w
        elif lane_num == 2:
            l_w = r_w = lane_width+h_w
        elif lane_num == 3:
            l_w = (2*lane_width)+h_w
            r_w = h_w
        elif lane_num == 4: # lane1 on 4lane
            l_w = h_w
            r_w = (3*lane_width)+h_w
        elif lane_num == 5: # lane2 on 4lane
            l_w = (1*lane_width)+h_w
            r_w = (2*lane_width)+h_w
        elif lane_num == 6: # lane3 on 4lane
            l_w = (2*lane_width)+h_w
            r_w = (1*lane_width)+h_w
        elif lane_num == 7: # lane4 on 4lane
            l_w = (3*lane_width)+h_w
            r_w = h_w
        else:
            l_w = r_w = h_w
    else:
        l_w = r_w = h_w

    return l_w, r_w
    

def PathViz(waypoints, color):
    return Path(waypoints, 999, 0.2, 1.5, color)

def PreRound2Viz(waypoints):
    return Path(waypoints, 999, 0.2, 1.5, (150/255,59/255, 255/255, 0.5))


def Path(waypoints, id_, z, scale, color):
    marker = Line('path', int(id_), scale, color, len(waypoints))
    for pt in waypoints:
        marker.points.append(Point(x=pt[0], y=pt[1], z=z))
    return marker

def Line(ns, id_, scale, color, len):
    marker = Marker()
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    return marker
    
def VelProfileViz(waypoints, vel_profile):
    marker_array = MarkerArray()
    for idx, (pt, vel) in enumerate(zip(waypoints, vel_profile)):
        marker_array.markers.append(TextMarker(pt, vel, idx+1000, 1, (1.0,1,1, 1.0)))
    return marker_array

def TextMarker(pt, vel, id_, scale, color):
    marker = Marker()
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = 'vel_text'
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.z = scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.pose.position.x = pt[0]
    marker.pose.position.y = pt[1]
    marker.pose.position.z = 2 # slightly above the waypoint
    marker.text = f"{vel:.2f} m/s"
    return marker

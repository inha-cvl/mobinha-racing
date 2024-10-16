import numpy as np
import math
import copy

from scipy.interpolate import splprep, splev, interp1d


def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def find_closest_index(global_path, local_pos, threshold=20):
    min_dist = float('inf')
    closest_index = None
    decreasing = True

    for i, point in enumerate(global_path):
        path_x = float(point[0])
        path_y = float(point[1])
        dist = distance(path_x, path_y, local_pos[0], local_pos[1])

        if dist < min_dist:
            min_dist = dist
            closest_index = i
            decreasing = True
        elif decreasing and dist > min_dist + threshold:
            decreasing = False
            break

    return closest_index


def generate_points(x_norm, y_norm, l_width):
    x_range = np.arange(x_norm - l_width, x_norm + l_width + 1, 2)
    y_range = np.arange(y_norm - l_width, y_norm + l_width + 1, 2)
    points = np.array([(x, y) for x in x_range for y in y_range])
    return points

def find_minimum_point(x_norm, y_norm, l_width):
    points = generate_points(x_norm, y_norm, l_width)
    point_sums = points[:, 0] + points[:, 1]
    min_index = np.argmin(point_sums)
    min_point = points[min_index]
    return min_point

def trim_and_update_global_path(global_path, local_pos, local_path_length):
    now_idx = find_closest_index(global_path, local_pos)
    end_idx = min(now_idx + local_path_length, len(global_path))
    copy_g_path = copy.deepcopy(global_path)
    trim_global_path = copy_g_path[now_idx:end_idx]
    updated_global_path = global_path[now_idx:]
    
    return trim_global_path, updated_global_path

def object2frenet(trim_path, obs_pose):

    centerline = np.array([(point[0], point[1]) for point in trim_path])
    point = np.array(obs_pose)

    tangents = np.gradient(centerline, axis=0)
    tangents = tangents / np.linalg.norm(tangents, axis=1)[:, np.newaxis]
    
    normals = np.column_stack([-tangents[:, 1], tangents[:, 0]])
    
    distances = np.linalg.norm(centerline - point, axis=1)
    
    closest_index = np.argmin(distances)
    closest_point = centerline[closest_index]
    tangent = tangents[closest_index]
    normal = normals[closest_index]
    
    vector_to_point = point - closest_point
    d = np.dot(vector_to_point, normal)

    # 경로 상의 거리를 계산 (경로 시작부터 closest_index까지의 거리)
    s = np.sum(np.linalg.norm(np.diff(centerline[:closest_index + 1], axis=0), axis=0))

    vector_from_start = point - centerline[0]  
    if np.dot(tangents[0], vector_from_start) < 0:  
        s = -np.linalg.norm(vector_from_start)  
    
    return s, d


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

def calculate_R_first_index(points):
    R = 0
    num_points = len(points)
    if num_points >= 10:
        epoints = points[0]
        npoints = [points[4], points[9]]
        kappa = calc_kappa(epoints, npoints)
        R = 1/kappa if kappa != 0 else 9999
    return abs(R)

def calculate_R_list(points, base_offset=2, step_size=40):
    Rs = []
    numpoints = len(points)
    last_R = 99999
    last_offset = step_size * 2

    for i in range(numpoints):
        if i + base_offset < numpoints - last_offset:
            epoints = points[i + base_offset]
            npoints = [points[i + base_offset + step_size], points[i + base_offset + 2 * step_size]]
            kappa = calc_kappa(epoints, npoints)
            R = abs(1 / kappa) if kappa != 0 else 99999
            last_R = R
            Rs.append(R)
        else:
            Rs.append(last_R)
    return Rs

def calculate_R_list2(array, base_offset=2, step_size=40):
    Rs = []
    numpoints = len(array)
    last_R = 99999
    last_offset = step_size * 2

    # array는 shape이 [n, 2]로 x 좌표는 array[:, 0], y 좌표는 array[:, 1]에 들어가 있다고 가정
    x_points = array[:, 0]
    y_points = array[:, 1]

    for i in range(numpoints):
        if i + base_offset < numpoints - last_offset:
            epoints = [x_points[i + base_offset], y_points[i + base_offset]]
            npoints = [[x_points[i + base_offset + step_size], y_points[i + base_offset + step_size]],
                       [x_points[i + base_offset + 2 * step_size], y_points[i + base_offset + 2 * step_size]]]
            kappa = calc_kappa(epoints, npoints)
            R = abs(1 / kappa) if kappa != 0 else 99999
            last_R = R
            Rs.append(R)
        else:
            Rs.append(last_R)
    return Rs

#best
def interpolate_path(final_global_path, min_length=100, sample_rate=4, smoothing_factor=50, interp_points=4):
    local_path = np.array([(point[0], point[1]) for point in final_global_path])
    local_vel = [point[10] for point in final_global_path]

    if len(local_path) > min_length:
        sampled_indices = np.arange(0, len(local_path), sample_rate)
        sampled_local_path = local_path[sampled_indices]
        sampled_local_vel = np.array(local_vel)[sampled_indices]

        tck, u = splprep([sampled_local_path[:, 0], sampled_local_path[:, 1]], s=smoothing_factor)
        t_new = np.linspace(0, 1, len(sampled_local_path) * interp_points)
        path_interp = np.array(splev(t_new, tck)).T
        path_interp_list = path_interp.tolist()
        vel_interp_func = interp1d(np.linspace(0, 1, len(sampled_local_vel)), sampled_local_vel, kind='linear')
        vel_interp = vel_interp_func(t_new).tolist()

        R_list = calculate_R_list(path_interp_list)
    else:
        path_interp_list = local_path.tolist()
        R_list = calculate_R_list(path_interp_list)
        vel_interp = local_vel

    return path_interp_list, R_list, vel_interp

def calc_overtaking_by_ttc(obj_dist, obj_vel, ego_vel,max_th= 15):
    rel_vel = ego_vel-obj_vel
    if rel_vel > 0:
        ttc = obj_dist/rel_vel
    else:
        ttc = float('inf')
    if (ttc <= max_th) or obj_vel < 2:
        return True
    else:
        return False

def calc_ttc(obj_dist, obj_vel, ego_vel):
    rel_vel = ego_vel-obj_vel
    if rel_vel > 0:
        ttc = obj_dist/rel_vel
    else:
        ttc = float('inf')
    
    return ttc

def check_bsd(left_bsd, right_bsd, lc_state):
    if lc_state == 'left':
        if left_bsd == 1:
            return True
        else:
            return False
    elif lc_state == 'right':
        if right_bsd == 1:
            return True
        else:
            return False
    else:
        return False

def get_lane_change_state(d, l_width, r_width):
    # d : left + , right -
    if l_width > r_width:
        if (l_width - d) > 1.5:
            lane_change_state = ['left']
            if abs(-r_width-d) > 1.5:
                lane_change_state.append('right')
        else:
            if abs(-r_width-d) > 1.5:
                lane_change_state=['right']
            else:
                lane_change_state = None
    else:
        if abs(-r_width-d) > 1.5:
            lane_change_state = ['right']
            if (l_width - d) > 1.5:
                lane_change_state.append('left')
        else:
            if (l_width - d) > 1.5:
                lane_change_state = ['left']
            else:
                lane_change_state = None
    return lane_change_state


def check_around(xobj, yobjs, lc_state, radius = 15):
    around = False
    for yobj in yobjs:
        if xobj['s'] - radius < yobj['s'] < xobj['s'] + radius:

            if lc_state == 'left':
                if yobj['d'] > xobj['d']:
                    around = True
            else:
                if yobj['d'] < xobj['d']:
                    around = True
    return around

def get_stop_distance(current_velocity, decel_factor=2.7):
    react_distance = current_velocity * 2
    brake_distance = (current_velocity)**2/(2*decel_factor)
    return react_distance + brake_distance

def check_lane_deaprture(local_path, localpos):
    if local_path is not None and len(local_path) > 0:
        dist = distance(local_path[0][0], local_path[0][1], localpos[0], localpos[1])
        if dist <= 5:
            return 'Normal'
        elif 5 < dist < 10:
            return 'Warning'
        else:
            return 'Danger'

def get_lr_threshold(trim_global_path, s):
    if s > 0:
        if s < len(trim_global_path)-1:
            l_th = trim_global_path[int(s)][3]
            r_th = -trim_global_path[int(s)][2]
        else:
            l_th = trim_global_path[-1][3]
            r_th = -trim_global_path[-1][2]
    else:
        l_th = trim_global_path[0][3]
        r_th = -trim_global_path[0][2]
    return l_th, r_th


def has_different_lane_number(prev, current):
    if prev != current:
        return True
    else:
        return False

def check_avoidance_gap_over(lc_state, l_width, r_width, lat_avoidance_gap, d):
    overed = True
    if lc_state == 'left':
        avoidance_gap = lat_avoidance_gap+d
        if avoidance_gap < l_width:
            overed = False
    else:
        avoidance_gap = lat_avoidance_gap-d
        if avoidance_gap < r_width:
            overed = False
    return overed, avoidance_gap

def get_selected_lane(max_vel):
    if max_vel < 70/3.6:
        return 3
    elif 70/3.6 <= max_vel < 95/3.6:
        return 2
    else:
        return 1
import numpy as np
import math
import copy

from scipy.interpolate import splprep, splev, interp1d

def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def find_closest_index(global_path, local_pos):
    min_dist = float('inf')
    closest_index = None
    for i, point in enumerate(global_path):
        path_x = float(point[0])
        path_y = float(point[1])
        dist = distance(path_x,path_y,local_pos[0],local_pos[1])
        if dist < min_dist:
            min_dist = dist
            closest_index = i
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
    s = np.sum(np.linalg.norm(np.diff(centerline[:closest_index + 1], axis=0), axis=0))
    
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

def calculate_R_list(points, base_offset=3, step_size=5):
    Rs = []
    numpoints = len(points)
    last_R = None
    last_offset = step_size * 2

    for i in range(numpoints):
        if i + base_offset < numpoints - last_offset:
            epoints = points[i + base_offset]
            npoints = [points[i + base_offset + step_size], points[i + base_offset + 2 * step_size]]
            kappa = calc_kappa(epoints, npoints)
            R = 1 / kappa if kappa != 0 else 99999
            last_R = R
            Rs.append(R)
        else:
            Rs.append(last_R)
    return Rs

def interpolate_path(final_global_path, min_length=100, sample_rate=2, smoothing_factor=7.0, interp_points=10):
    local_path = np.array([(point[0], point[1]) for point in final_global_path])
    local_vel = [point[10] for point in final_global_path]

    if len(local_path) > min_length:
        sampled_indices = np.arange(0, len(local_path), sample_rate)
        sampled_local_path = local_path[sampled_indices]
        sampled_local_vel = np.array(local_vel)[sampled_indices]
        # sampled_local_path = local_path
        # sampled_local_vel = local_vel

        tck, u = splprep([sampled_local_path[:, 0], sampled_local_path[:, 1]], s=smoothing_factor)
        t_new = np.linspace(0, 1, len(sampled_local_path) * interp_points)
        path_interp = np.array(splev(t_new, tck)).T
        path_interp_list = path_interp.tolist()
        vel_interp_func = interp1d(np.linspace(0, 1, len(sampled_local_vel)), sampled_local_vel, kind='linear')
        vel_interp = vel_interp_func(t_new).tolist()

        #first_R = calculate_R_first_index(path_interp)
        R_list = calculate_R_list(path_interp_list)
    else:
        path_interp_list = local_path.tolist()
        #first_R = calculate_R_first_index(path_interp_list)
        R_list = calculate_R_list(path_interp_list)
        vel_interp = local_vel

    return path_interp_list, R_list, vel_interp
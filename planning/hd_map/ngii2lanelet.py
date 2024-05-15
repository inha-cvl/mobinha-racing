import sys
import math
import utm
import pymap3d
import numpy as np
from tqdm import tqdm

from ngiiParser import NGIIParser
from quadratic_spline_interpolate import QuadraticSplineInterpolate

from shapely.geometry import LineString, Polygon
from scipy.interpolate import splprep, splev

def convert_2_360(angle):
    if angle >= 0 and angle <= math.pi:
        return angle
    else:
        return math.radians(360) + angle


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def get_yaw_error(yaw, angle):
    angle1 = convert_2_360(yaw)
    angle2 = convert_2_360(angle)
    return pi_2_pi(angle1 - angle2)


def euc_distance(pt1, pt2):
    return np.sqrt((pt2[0]-pt1[0])**2+(pt2[1]-pt1[1])**2)


def find_nearest_idx(pts, pt):
    min_dist = sys.maxsize
    min_idx = 0

    for idx, pt1 in enumerate(pts):
        dist = euc_distance(pt1, pt)
        if dist < min_dist:
            min_dist = dist
            min_idx = idx

    return min_idx


def interpolate(points, precision):
    def filter_same_points(points):
        filtered_points = []
        pre_pt = None

        for pt in points:
            if pre_pt is None or pt != pre_pt:
                filtered_points.append(pt)

            pre_pt = pt

        return filtered_points

    points = filter_same_points(points)
    if len(points) < 2:
        return points, None, None, None

    wx, wy = zip(*points)
    itp = QuadraticSplineInterpolate(list(wx), list(wy))

    itp_points = []
    s = []
    yaw = []
    k = []

    for n, ds in enumerate(np.arange(0.0, itp.s[-1], precision)):
        s.append(ds)
        x, y = itp.calc_position(ds)
        dyaw = itp.calc_yaw(ds)

        dk = itp.calc_curvature(ds)

        itp_points.append((float(x), float(y)))
        yaw.append(dyaw)
        k.append(dk)

    return itp_points, s, yaw, k


class NGII2LANELET:
    def __init__(self,
                 folder_path,
                 precision,
                 base_lla,
                 is_utm):

        a1_path = '%s/A1_NODE.shp'%(folder_path)
        a2_path = '%s/A2_LINK.shp'%(folder_path)
        b2_path = '%s/B2_SURFACELINEMARK.shp'%(folder_path)

        ngii = NGIIParser(
            a1_path,
            a2_path,
            b2_path)
                   
        self.base_lla = base_lla
        self.is_utm = is_utm
        self.generate_lanelet(ngii, precision, self.base_lla, self.is_utm)

    def to_cartesian(self, tx, ty, alt=None):
        if self.is_utm:
            lat, lon = utm.to_latlon(tx, ty, 52, 'N')
        else:
            lat, lon = ty, tx

        if self.base_lla is None:
            self.base_lla = (lat, lon, alt)

        if alt is None:
            x, y, _ = pymap3d.geodetic2enu(lat, lon, self.base_lla[2], self.base_lla[0], self.base_lla[1], self.base_lla[2])
            return x, y
        else:
            x, y, z = pymap3d.geodetic2enu(lat, lon, alt, self.base_lla[0], self.base_lla[1], self.base_lla[2])
            return x, y, z

    def generate_lanelet(self, ngii, precision, base_lla, is_utm):
        self.link_id_data = {}
        self.map_data = {}
        lanelets = {}

        for_vis = []

        ori2new = {}
        self.new2ori = {}

        to_node = {}
        from_node = {}

        stoplines = {}
        safetysigns = {}

        for n, a2_link in tqdm(enumerate(ngii.a2_link), desc="a2_link: ", total=len(ngii.a2_link)):
            if a2_link.Length == 0:
                continue

            new_id = str(n)
            ori_id = a2_link.ID
            ori2new[ori_id] = new_id
            self.new2ori[new_id] = ori_id

            lanelets[new_id] = {}

            waypoints = []
            
            for tx, ty, alt in a2_link.geometry.coords:
                x, y, z = self.to_cartesian(tx, ty, alt)
                waypoints.append((x, y))

            waypoints, s, yaw, k = interpolate(waypoints, precision)

            lanelets[new_id]['waypoints'] = waypoints
            lanelets[new_id]['idx_num'] = len(waypoints)
            lanelets[new_id]['yaw'] = yaw
            lanelets[new_id]['s'] = s
            lanelets[new_id]['k'] = k
            lanelets[new_id]['length'] = s[-1]  # a2_link.length
            lanelets[new_id]['laneNo'] = a2_link.LaneNo
            
            lanelets[new_id]['leftTurn'] = False
            lanelets[new_id]['rightTurn'] = False
            lanelets[new_id]['uTurn'] = False
            lanelets[new_id]['direction'] = []
            lanelets[new_id]['trafficLight'] = []
            lanelets[new_id]['stoplineID'] = []
            lanelets[new_id]['crosswalkID'] = []

            lanelets[new_id]['leftBound'] = []
            lanelets[new_id]['leftType'] = []
            lanelets[new_id]['rightBound'] = []
            lanelets[new_id]['rightType'] = []


            if to_node.get(a2_link.ToNodeID) is None:
                to_node[a2_link.ToNodeID] = []

            to_node[a2_link.ToNodeID].append(new_id)

            if from_node.get(a2_link.FromNodeID) is None:
                from_node[a2_link.FromNodeID] = []

            from_node[a2_link.FromNodeID].append(new_id)


            if a2_link.LinkType == '1':
                lanelets[new_id]['intersection'] = True
            else:
                lanelets[new_id]['intersection'] = False

            if str(a2_link.LaneNo)[0] == '9':
                lanelets[new_id]['leftTurn'] = True
            else:
                lanelets[new_id]['leftTurn'] = False

            if a2_link.MaxSpeed is None or int(a2_link.MaxSpeed) == 0.0:
                lanelets[new_id]['speedLimit'] = 50
            else:
                lanelets[new_id]['speedLimit'] = int(a2_link.MaxSpeed)

        for a2_link in ngii.a2_link:
            if a2_link.Length == 0:
                continue

            ori_id = a2_link.ID
            new_id = ori2new[ori_id]
            lanelets[new_id]['adjacentLeft'] = ori2new.get(a2_link.L_LinkID)
            lanelets[new_id]['adjacentRight'] = ori2new.get(a2_link.R_LinkID)

            lanelets[new_id]['predecessor'] = to_node[a2_link.FromNodeID] if to_node.get(a2_link.FromNodeID) is not None else []
            lanelets[new_id]['successor'] = from_node[a2_link.ToNodeID] if from_node.get(a2_link.ToNodeID) is not None else []


        for id_, data in lanelets.items():
            left_id = data['adjacentLeft']
            
            if left_id is not None:
                left_data = lanelets[left_id]
                if left_data['adjacentRight'] != id_:
                    data['adjacentLeft'] = None

            right_id = data['adjacentRight']
            if right_id is not None:
                right_data = lanelets[right_id]
                if right_data['adjacentLeft'] != id_:
                    data['adjacentRight'] = None

        # Grouping
        groups = []
        g_closed = []
        for id_, data in tqdm(lanelets.items(), desc="not groups: ", total=len(groups)):
            if id_ not in g_closed:
                left_id = data['adjacentLeft']
                right_id = data['adjacentRight']

                if left_id is not None or right_id is not None:
                    group_n = len(groups)
                    lanelets[id_]['group'] = group_n
                    group = [id_]
                    g_closed.append(id_)
                    while left_id is not None:
                        lanelets[left_id]['group'] = group_n
                        group.insert(0, left_id)
                        g_closed.append(left_id)
                        left_id = lanelets[left_id]['adjacentLeft']

                    while right_id is not None:
                        lanelets[right_id]['group'] = group_n
                        group.append(right_id)
                        g_closed.append(right_id)
                        right_id = lanelets[right_id]['adjacentRight']

                    groups.append(group)

                else:
                    data['group'] = None
        

        for b2_surfacelinemark in tqdm(ngii.b2_surfacelinemark, desc="b2_surfacelinemark: ", total=len(ngii.b2_surfacelinemark)):
            if b2_surfacelinemark.Kind is not None:
                if b2_surfacelinemark.Kind != '530':
                    ## right link
                    ori_id = b2_surfacelinemark.R_LinkID
                    right_id = ori2new.get(ori_id)

                    if b2_surfacelinemark.geometry is not None:
                        leftBound = []
                        for tx, ty, alt in b2_surfacelinemark.geometry.coords:
                            x, y, z = self.to_cartesian(tx, ty, alt)
                            leftBound.append((x, y))

                        leftBound, s, yaw, k = interpolate(leftBound, precision)

                        if len(leftBound) > 1:
                            if right_id is not None:
                                lanelets[right_id]['leftBound'].append(leftBound)
                                lanelets[right_id]['leftType'].append('solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted')
                            else:
                                for_vis.append([leftBound, 'solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted'])
                    ## left link
                    ori_id = b2_surfacelinemark.L_LinkID
                    left_id = ori2new.get(ori_id)

                    if b2_surfacelinemark.geometry is not None:
                        rightBound = []
                        for tx, ty, alt in b2_surfacelinemark.geometry.coords:
                            x, y, z = self.to_cartesian(tx, ty, alt)
                            rightBound.append((x, y))

                        rightBound, s, yaw, k = interpolate(rightBound, precision)

                        if len(rightBound) > 1:
                            if left_id is not None:
                                lanelets[left_id]['rightBound'].append(rightBound)
                                lanelets[left_id]['rightType'].append('solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted')
                            else:
                                for_vis.append([rightBound, 'solid' if b2_surfacelinemark.Type[2] == '1' else 'dotted'])

                else:  # stop line
                    stopline_id = b2_surfacelinemark.ID
                    lines = []
                    for tx, ty, alt in b2_surfacelinemark.geometry.coords:
                        x, y, z = self.to_cartesian(tx, ty, alt)
                        lines.append((x, y))

                    stoplines[stopline_id] = lines
                    for_vis.append([lines, 'stop_line'])

        for id_, data in lanelets.items():
            if data['length'] < 35.0:
                yaw_err = get_yaw_error(data['yaw'][0], data['yaw'][-1])
                if abs(math.degrees(yaw_err)) > 150:
                    data['uTurn'] = True
            
        for id_, data in lanelets.items():
            data['leftChange'] = [True for _ in range(len(data['waypoints']))]
            data['rightChange'] = [True for _ in range(len(data['waypoints']))]

            for leftBound, leftType in zip(data['leftBound'], data['leftType']):
                if leftType == 'solid':
                    idx_s = find_nearest_idx(data['waypoints'], leftBound[0])
                    idx_f = find_nearest_idx(data['waypoints'], leftBound[-1])
                    data['leftChange'][idx_s:idx_f] = [
                        False for _ in range(idx_f-idx_s)]

            for rightBound, rightType in zip(data['rightBound'], data['rightType']):
                if rightType == 'solid':
                    idx_s = find_nearest_idx(data['waypoints'], rightBound[0])
                    idx_f = find_nearest_idx(data['waypoints'], rightBound[-1])
                    data['rightChange'][idx_s:idx_f] = [
                        False for _ in range(idx_f-idx_s)]

        for id_, data in lanelets.items():
            self.link_id_data[id_]=self.new2ori[id_]

        self.map_data['base_lla'] = base_lla
        self.map_data['precision'] = precision
        self.map_data['lanelets'] = lanelets
        self.map_data['groups'] = groups
        self.map_data['for_vis'] = for_vis
        self.map_data['stoplines'] = stoplines
        self.map_data['safetysigns'] = safetysigns
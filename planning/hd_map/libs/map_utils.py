import numpy as np
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from hd_map.libs.quadratic_spline_interpolate import QuadraticSplineInterpolate

def LaneletMapViz(lanelet, for_viz):
    array = MarkerArray()
    for id_, data in lanelet.items():
        for n, (leftBound, leftType) in enumerate(zip(data['leftBound'], data['leftType'])):
            marker = Bound('leftBound', id_, n, leftBound,
                           leftType, (1.0, 1.0, 1.0, 1.0))
            array.markers.append(marker)

        for n, (rightBound, rightType) in enumerate(zip(data['rightBound'], data['rightType'])):
            marker = Bound('rightBound', id_, n, rightBound,
                           rightType, (1.0, 1.0, 1.0, 1.0))
            array.markers.append(marker)

    for n, (points, type_) in enumerate(for_viz):
        if type_ == 'stop_line':
            marker = Bound('for_viz', n, n, points,
                           'solid', (1.0, 1.0, 1.0, 1.0))
            array.markers.append(marker)
        else:
            marker = Bound('for_viz', n, n, points,
                           type_, (1.0, 1.0, 1.0, 1.0))
            array.markers.append(marker)

    return array

def MicroLaneletGraphViz(lanelet, graph):
    array = MarkerArray()

    for n, (node_id, data) in enumerate(graph.items()):
        split = node_id.split('_')

        if len(split) == 1:
            id_ = split[0]
            from_idx = lanelet[id_]['idx_num'] // 2
            from_pts = lanelet[id_]['waypoints']
            marker = Node(node_id, n, from_pts[from_idx], (1.0, 1.0, 1.0, 1.0))
            array.markers.append(marker)

            for m, target_node_id in enumerate(data.keys()):
                split = target_node_id.split('_')
                pts = []

                if len(split) == 1:
                    target_id = split[0]
                    to_pts = lanelet[target_id]['waypoints']
                    to_idx = lanelet[target_id]['idx_num'] // 2
                    pts.extend(from_pts[from_idx:])
                    pts.extend(to_pts[:to_idx])
                else:
                    target_id = split[0]
                    cut_n = int(split[1])
                    to_pts = lanelet[target_id]['waypoints']
                    to_idx = sum(lanelet[target_id]['cut_idx'][cut_n]) // 2
                    pts.extend(from_pts[from_idx:])
                    pts.extend(to_pts[:to_idx])

                marker1, marker2 = Edge(n*100000+m, pts, (0.0, 1.0, 0.0, 0.5))
                array.markers.append(marker1)
                array.markers.append(marker2)

        else:
            id_ = split[0]
            cut_n = int(split[1])
            from_idx = sum(lanelet[id_]['cut_idx'][cut_n]) // 2
            from_pts = lanelet[id_]['waypoints']
            marker = Node(node_id, n, from_pts[from_idx], (1.0, 1.0, 1.0, 1.0))
            array.markers.append(marker)

            for m, target_node_id in enumerate(data.keys()):
                split = target_node_id.split('_')
                pts = []

                if len(split) == 1:
                    target_id = split[0]
                    to_pts = lanelet[target_id]['waypoints']
                    to_idx = lanelet[target_id]['idx_num'] // 2
                    pts.extend(from_pts[from_idx:])
                    pts.extend(to_pts[:to_idx])
                else:
                    target_id = split[0]
                    cut_n = int(split[1])
                    to_pts = lanelet[target_id]['waypoints']
                    to_idx = sum(lanelet[target_id]['cut_idx'][cut_n]) // 2
                    pts = [from_pts[from_idx], to_pts[to_idx]]

                
                marker1, marker2 = Edge(n*100000+m, pts, (0.0, 1.0, 0.0, 0.3))
                array.markers.append(marker1)
                array.markers.append(marker2)

    return array

def Bound(ns, id_, n, points, type_, color):
    if type_ == 'solid':
        marker = Line('%s_%s' % (ns, id_), n, 0.15, color, 0)
        for pt in points:
            marker.points.append(Point(x=pt[0], y=pt[1], z=0.0))

    elif type_ == 'dotted':
        marker = Points('%s_%s' % (ns, id_), n, 0.15, color)
        for pt in points:
            marker.points.append(Point(x=pt[0], y=pt[1], z=0.0))

    return marker

def Node(id_, n, pt, color):
    marker = Text('graph_id', n, 1.5, color, id_)
    marker.pose.position = Point(x=pt[0], y=pt[1], z=1.0)
    return marker

def Edge(n, points, color):

    if len(points) == 2:
        wx, wy = zip(*points)
        itp = QuadraticSplineInterpolate(list(wx), list(wy))
        pts = []
        for ds in np.arange(0.0, itp.s[-1], 0.5):
            pts.append(itp.calc_position(ds))
        points = pts


    marker1 = Line('edge_line', n, 0.2, color, 0)
    for pt in points:
        marker1.points.append(Point(x=pt[0], y=pt[1], z=0.0))

    marker2 = Arrow('edge_arrow', n, (0.3, 0.8, 1.0), color)
    num = len(points)
    if num > 2:
        marker2.points.append(
            Point(x=points[-min(max(num, 3), 5)][0], y=points[-min(max(num, 3), 5)][1]))
    else:
        marker2.points.append(Point(x=points[-2][0], y=points[-2][1]))
    marker2.points.append(Point(x=points[-1][0], y=points[-1][1]))
    return marker1, marker2


def Points(ns, id_, scale, color):
    marker = Marker()
    marker.type = Marker.POINTS
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = scale
    marker.scale.y = scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
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

def Text(ns, id_, scale, color, text):
    marker = Marker()
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.text = text
    marker.scale.z = scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    return marker

def Arrow(ns, id_, scale, color):
    marker = Marker()
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.header.frame_id = 'world'
    marker.ns = ns
    marker.id = id_
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    return marker

def find_nearest_idx(pts, pt):
    min_dist = float('inf')
    min_idx = 0

    for idx, pt1 in enumerate(pts):
        dist = euc_distance(pt1, pt)
        if dist < min_dist:
            min_dist = dist
            min_idx = idx

    return min_idx

def euc_distance(pt1, pt2):
    return np.sqrt((pt2[0]-pt1[0])**2+(pt2[1]-pt1[1])**2)
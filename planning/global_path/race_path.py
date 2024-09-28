#!/usr/bin/env python3
import os
import sys
toppath = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(toppath)

import copy
import rospy
from visualization_msgs.msg import Marker
import global_path
import global_path.libs.gp_utils as gput

rospy.init_node('GlobalPath', anonymous=False)
target = 'race'
pub_global_path = rospy.Publisher(f'/{target}_path', Marker, queue_size=1)

diag_len = 25
lane_width = 3.25

map = global_path.libs.load_map.MAP('KIAPI_Racing_Fast')
gput.lanelets = map.lanelets
gput.tiles = map.tiles
gput.tile_size = map.tile_size
gput.graph = map.graph
gput.lane_width = lane_width

start_pose =  [504.961, -415.803] #[523.224, -441.700]
final_path = []
final_ids = []
final_vs = []


start_ll = gput.lanelet_matching(start_pose)
r1, idnidx1, ids1, vs1 = gput.get_straight_path(start_ll, 3800, '77', 'Right') #77 , 61
# idnidx2 = gput.get_merged_point(idnidx1, diag_len, 1)
# r2, idnidx3, ids2, vs2 = gput.get_straight_path(idnidx2, 5000, '76', 'Right')
# idnidx4 = gput.get_merged_point(idnidx3, diag_len, 2)
# r3, idnidx3, ids3, vs3 = gput.get_straight_path(idnidx4, 7000, '76', 'Right') 


final_path = r1[:-200]#+r2#+r3
final_ids = ids1[:-200]#+ids2#+ids3
final_vs = vs1[:-200]#+vs2#+vs3
start_pose = r1[-201]

final_tr = []
copy_final_path = copy.deepcopy(final_path)
# copy_final_path.insert(0, final_path[0])
# copy_final_path.append(final_path[-1])


vel_p, accel_p, dist_p = gput.adjust_velocity_profile(final_vs)
s = 0
window_size = 15
for i,f in enumerate(final_path):
    #before_after_pts = [copy_final_path[i], copy_final_path[i+2]]
    before_after_pts = [copy_final_path[max(0,i-window_size)], copy_final_path[min(len(final_path)-1,i+window_size)]]
    lw_left, lw_right = gput.get_lane_width(final_ids[i])
    A, B, theta = gput.calc_norm_vec(before_after_pts)
    Rk = gput.calc_kappa_spline(f, before_after_pts)
    vx = vel_p[i]
    ax = accel_p[i]
    final_tr.append([f[0], f[1], lw_right, lw_left, A, B, 0, s, theta, Rk, vx, ax])
    s += 1

# radii = gput.compute_curvature_radius(final_path)
# for i in range(len(final_tr)):
#     final_tr[i][9] = radii[i]  # 9번째 인덱스는 ""로 비워둔 자리


top_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
file_path = f'{top_path}/inputs/traj_ltpl_cl/traj_ltpl_cl_{target}.csv'
global_path.libs.save_.to_csv(file_path, final_tr)
print("saved")

color = (150/255,59/255, 255/255, 0.5)
global_path_viz = gput.PathViz(final_path, color)

rate = rospy.Rate(0.5)
while not rospy.is_shutdown():
    pub_global_path.publish(global_path_viz)
    rate.sleep()
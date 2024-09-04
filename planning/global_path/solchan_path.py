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
target = 'to_goal_solchan'
pub_global_path1 = rospy.Publisher(f'/{target}_path', Marker, queue_size=1)


diag_len = 20
lane_width = 3.25

map = global_path.libs.load_map.MAP('Solchan')
gput.lanelets = map.lanelets
gput.tiles = map.tiles
gput.tile_size = map.tile_size
gput.graph = map.graph
gput.lane_width = lane_width

start_pose = [-0.955, -1.030]
final_path = []
final_ids = []
final_vs = []


start_ll = gput.lanelet_matching(start_pose)
r1, idnidx1, ids1, vs1 = gput.get_straight_path(start_ll, 90000, '93', 'Right')
idnidx2 = gput.get_merged_point(idnidx1, diag_len, 1)
r2, idnidx3, ids2, vs2 = gput.get_straight_path(idnidx2, 10000, '54', 'Right') 
idnidx4 = gput.get_merged_point(idnidx3, diag_len, 1)
r3, idnidx5, ids3, vs3 = gput.get_straight_path(idnidx4, 10000, '49', 'Right') 
idnidx6 = gput.get_merged_point(idnidx5, diag_len, 1)
r4, idnidx7, ids4, vs4 = gput.get_straight_path(idnidx6, 10000, '162', 'Right') 
idnidx8 = gput.get_merged_point(idnidx7, diag_len, 1)
r5, idnidx9, ids5, vs5 = gput.get_straight_path(idnidx8, 10000, '179', 'Right') 



final_path = r1+r2+r3+r4+r5
final_ids = ids1+ids2+ids3+ids4+ids5
final_vs = vs1+vs2+vs3+vs4+vs5
start_pose = r5[-1]

final_tr = []
copy_final_path = copy.deepcopy(final_path)
copy_final_path.insert(0, final_path[0])
copy_final_path.append(final_path[-1])

vel_p, accel_p, dist_p = gput.adjust_velocity_profile(final_vs)
s = 0

for i,f in enumerate(final_path):
    before_after_pts = [copy_final_path[i], copy_final_path[i+2]]
    lw_left, lw_right = gput.get_lane_width(final_ids[i])
    A, B, theta = gput.calc_norm_vec(before_after_pts)
    Rk = gput.calc_kappa(f, before_after_pts)
    vx = vel_p[i]
    ax = accel_p[i]
    final_tr.append([f[0], f[1], lw_right, lw_left, A, B, 0, s, theta, Rk, vx, ax])
    s += 1

top_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
file_path = f'{top_path}/inputs/traj_ltpl_cl/traj_ltpl_cl_{target}.csv'
global_path.libs.save_.to_csv(file_path, final_tr)

print(f"{target} saved")

color = (255/255,79/255, 66/255, 0.5)
global_path_viz1 = gput.PathViz(final_path, color)


# ------------------------------------------------------------------------------------

target = 'race_solchan'
pub_global_path2 = rospy.Publisher(f'/{target}_path', Marker, queue_size=1)

start_pose = [749.531, -512.807]
final_path = []
final_ids = []
final_vs = []


start_ll = gput.lanelet_matching(start_pose)
r1, idnidx1, ids1, vs1 = gput.get_straight_path(start_ll, 90000, '195', 'Right')
idnidx2 = gput.get_merged_point(idnidx1, diag_len, 1)
r2, idnidx3, ids2, vs2 = gput.get_straight_path(idnidx2, 10000, '201', 'Right') 
idnidx4 = gput.get_merged_point(idnidx3, diag_len, 1)
r3, idnidx5, ids3, vs3 = gput.get_straight_path(idnidx4, 10000, '54', 'Right') 
idnidx6 = gput.get_merged_point(idnidx5, diag_len, 1)
r4, idnidx7, ids4, vs4 = gput.get_straight_path(idnidx6, 10000, '49', 'Right') 
idnidx8 = gput.get_merged_point(idnidx7, diag_len, 1)
r5, idnidx9, ids5, vs5 = gput.get_straight_path(idnidx8, 10000, '81', 'Right') 
idnidx10 = gput.get_merged_point(idnidx9, diag_len, 1)
r6, idnidx11, ids6, vs6 = gput.get_straight_path(idnidx10, 10000, '179', 'Right') 



final_path = r1+r2+r3+r4+r5+r6
final_ids = ids1+ids2+ids3+ids4+ids5+ids6
final_vs = vs1+vs2+vs3+vs4+vs5+vs6

final_tr = []
copy_final_path = copy.deepcopy(final_path)
copy_final_path.insert(0, final_path[0])
copy_final_path.append(final_path[-1])

vel_p, accel_p, dist_p = gput.adjust_velocity_profile(final_vs)
s = 0

for i,f in enumerate(final_path):
    before_after_pts = [copy_final_path[i], copy_final_path[i+2]]
    lw_left, lw_right = gput.get_lane_width(final_ids[i])
    A, B, theta = gput.calc_norm_vec(before_after_pts)
    Rk = gput.calc_kappa(f, before_after_pts)
    vx = vel_p[i]
    ax = accel_p[i]
    final_tr.append([f[0], f[1], lw_right, lw_left, A, B, 0, s, theta, Rk, vx, ax])
    s += 1

top_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
file_path = f'{top_path}/inputs/traj_ltpl_cl/traj_ltpl_cl_{target}.csv'
global_path.libs.save_.to_csv(file_path, final_tr)

print(f"{target} saved")


color = (150/255,59/255, 255/255, 0.5)
global_path_viz2 = gput.PathViz(final_path, color)



# Visualize ------------------------------------------------------------------------------------


rate = rospy.Rate(0.5)
while not rospy.is_shutdown():
    pub_global_path2.publish(global_path_viz2)
    rate.sleep()


'''

# target = 'solchan_lane2'
# pub_global_path2 = rospy.Publisher(f'/{target}_path', Marker, queue_size=1)

# start_pose = [33.170, 23.609]
# final_path = []
# final_ids = []
# final_vs = []


# start_ll = gput.lanelet_matching(start_pose)
# r1, idnidx1, ids1, vs1 = gput.get_straight_path(start_ll, 4000, '13', 'Right')
# idnidx2 = gput.get_merged_point(idnidx1, diag_len, 1)
# r2, idnidx3, ids2, vs2 = gput.get_straight_path(idnidx2, 5000, '54', 'Right') 
# idnidx4 = gput.get_merged_point(idnidx3, diag_len, 1)
# r3, idnidx5, ids3, vs3 = gput.get_straight_path(idnidx4, 5000, '49', 'Right') 
# idnidx6 = gput.get_merged_point(idnidx5, diag_len, 1)
# r4, idnidx7, ids4, vs4 = gput.get_straight_path(idnidx6, 5000, '39', 'Right') 
# idnidx8 = gput.get_merged_point(idnidx7, diag_len, 1)
# r5, idnidx9, ids5, vs5 = gput.get_straight_path(idnidx8, 10000, '307', 'Right') 
# # idnidx10 = gput.get_merged_point(idnidx9, diag_len, 1)
# # r6, idnidx11, ids6, vs6 = gput.get_straight_path(idnidx10, 10000, '179', 'Right') 



# final_path = r1+r2+r3+r4+r5#+r6
# final_ids = ids1+ids2+ids3+ids4+ids5#+ids6
# final_vs = vs1+vs2+vs3+vs4+vs5#+vs6

# final_tr = []
# copy_final_path = copy.deepcopy(final_path)
# copy_final_path.insert(0, final_path[0])
# copy_final_path.append(final_path[-1])

# vel_p, accel_p, dist_p = gput.adjust_velocity_profile(final_vs)
# s = 0

# for i,f in enumerate(final_path):
#     before_after_pts = [copy_final_path[i], copy_final_path[i+2]]
#     lw_left, lw_right = gput.get_lane_width(final_ids[i])
#     A, B, theta = gput.calc_norm_vec(before_after_pts)
#     Rk = gput.calc_kappa(f, before_after_pts)
#     vx = vel_p[i]
#     ax = accel_p[i]
#     final_tr.append([f[0], f[1], lw_right, lw_left, A, B, 0, s, theta, Rk, vx, ax])
#     s += 1

# top_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
# file_path = f'{top_path}/inputs/traj_ltpl_cl/traj_ltpl_cl_{target}.csv'
# global_path.libs.save_.to_csv(file_path, final_tr)

# print(f"{target} saved")


# color = (150/255,59/255, 255/255, 0.5)
# global_path_viz2 = gput.PathViz(final_path, color)

'''
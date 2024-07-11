import rospy
import copy
import numpy as np
from visualization_msgs.msg import Marker

from libs.lanelet import LaneletMap, TileMap
from libs.micro_lanelet_graph import MicroLaneletGraph
import libs.gp_utils as gput
import libs.save_ as save_

rospy.init_node('Solchan', anonymous=False)
pub_pr1_path = rospy.Publisher('/solchan_path', Marker, queue_size=1)

start_index = 0 # 0 : lane 2, 1 : lane 4
tile_size = 5
cut_dist = 15
precision = 1
diag_len = 25
lane_width = 3.25
max_vel = 80 / 3.6
sec_to_reach = 7
dist_to_reach_max_vel = int((0.5*(max_vel/sec_to_reach)*(sec_to_reach**2)))

lmap = LaneletMap('/home/hsy/catkin_ws/src/mobinha-racing/map_lane/hd_map/maps/Solchan.json')
tmap = TileMap(lmap.lanelets, tile_size)
mlg = MicroLaneletGraph(lmap, cut_dist)
graph = mlg.graph

gput.lanelets = mlg.lanelets
gput.tiles = tmap.tiles
gput.tile_size = tile_size
gput.lane_width = lane_width
start_pose = [1311.442, -958.255]
final_path = []
final_ids = []
final_vs = []

for i in range(1,6):
    start_ll = gput.lanelet_matching(start_pose)
    r1, idnidx1, ids1, vs1 = gput.get_straight_path(start_ll, 500, '359', 'Right')
    idnidx2 = gput.get_merged_point(idnidx1, diag_len, 1)
    r2, idnidx3, ids2, vs2 = gput.get_straight_path(idnidx2, 2000, '463', 'Right') 
    idnidx4 = gput.get_merged_point(idnidx3, diag_len, 1)
    r3, idnidx5, ids3, vs3 = gput.get_straight_path(idnidx4, 1000, '421', 'Right')
    idnidx6 = gput.get_pocket_successor(idnidx5[0], 'Left')
    r4, idnidx7, ids4, vs4 = gput.get_straight_path(idnidx6, 300, '411', 'Right')
    idnidx8 = gput.get_merged_point(idnidx7, diag_len, 1)
    r5, idnidx9, ids5, vs5 = gput.get_straight_path(idnidx8, 1000, '540', 'Right')
    idnidx10 = gput.get_merged_point(idnidx9, diag_len, 1)
    r6, idnidx11, ids6, vs6 = gput.get_straight_path(idnidx10, 500, '585', 'Right')
    idnidx12 = gput.get_pocket_successor(idnidx11[0], 'Left')
    r7, idnidx13, ids7, vs7 = gput.get_straight_path(idnidx12, 100, '573', 'Right')

    lap_path = r1+r2+r3+r4+r5+r6+r7
    lap_ids = ids1+ids2+ids3+ids4+ids5+ids6+ids7
    lap_vs = vs1+vs2+vs3+vs4+vs5+vs6+vs7
    start_pose = r7[-1] 

    final_path.extend(lap_path)
    final_ids.extend(lap_ids)
    final_vs.extend(lap_vs)

final_tr = []
copy_final_path = copy.deepcopy(final_path)
copy_final_path.insert(0, final_path[0])
copy_final_path.append(final_path[-1])

vel_p, accel_p, dist_p = gput.adjust_velocity_profile(final_vs)
# accel_p, vel_p, dist_p = gput.get_profiles(final_vs, sec_to_reach)
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

save_.to_csv('./paths/Solchan.csv', final_tr)
save_.to_txt('./ids/Solchan_id.txt', final_ids)

print("saved")
pr1_path_viz = gput.PreRound1Viz(final_path)

rate = rospy.Rate(0.5)
while not rospy.is_shutdown():
    pub_pr1_path.publish(pr1_path_viz)
    rate.sleep()
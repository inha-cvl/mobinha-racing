import rospy
import copy
import numpy as np
from visualization_msgs.msg import Marker

from libs.lanelet import LaneletMap, TileMap
from libs.micro_lanelet_graph import MicroLaneletGraph
import libs.gp_utils as gput
import libs.save_ as save_

rospy.init_node('Solbat', anonymous=False)
pub_pr1_path = rospy.Publisher('/solbat_path', Marker, queue_size=1)

start_index = 0 # 0 : lane 2, 1 : lane 4
tile_size = 5
cut_dist = 15
precision = 1
diag_len = 25
lane_width = 3.25
max_vel = 80 / 3.6
sec_to_reach = 7
dist_to_reach_max_vel = int((0.5*(max_vel/sec_to_reach)*(sec_to_reach**2)))

lmap = LaneletMap('/home/kana/catkin_ws/src/mobinha-racing/map_lane/hd_map/maps/Solbat.json')
tmap = TileMap(lmap.lanelets, tile_size)
mlg = MicroLaneletGraph(lmap, cut_dist)
graph = mlg.graph

gput.lanelets = mlg.lanelets
gput.tiles = tmap.tiles
gput.tile_size = tile_size
gput.lane_width = lane_width
start_pose = [0,0]
final_path = []
final_ids = []
final_vs = []


# 3 -> 2 -> 1 -> 2 -> 2
#LAP 1
start_ll = gput.lanelet_matching(start_pose)
r1, idnidx1, ids1, vs1 = gput.get_straight_path(start_ll, 900, '731', 'Right')
idnidx2 = gput.get_merged_point(idnidx1, diag_len, 1)
r2, idnidx2, ids2, vs2 = gput.get_straight_path(idnidx2, 7000, '372', 'Right') 

lap1_path = r1+r2
lap1_ids = ids1+ids2
lap1_vs = vs1+vs2
start_pose = r2[-1]

# #LAP 2
# start_ll = gput.lanelet_matching(start_pose)
# r1, idnidx1, ids1, vs1 = gput.get_straight_path(start_ll, 400, '62', 'Right')
# idnidx2 = gput.get_merged_point(idnidx1, diag_len, 1)
# r2, idnidx3, ids2, vs2 = gput.get_straight_path(idnidx2, 9000, '19', 'Right') 

# lap2_path = r1+r2
# lap2_ids = ids1+ids2
# lap2_vs = vs1+vs2
# start_pose = r2[-1]

# #LAP 3
# start_ll = gput.lanelet_matching(start_pose)
# r1, idnidx1, ids1, vs1 = gput.get_straight_path(start_ll, 400, '61', 'Right')
# idnidx2 = gput.get_merged_point(idnidx1, diag_len, 1)
# r2, idnidx3, ids2, vs2 = gput.get_straight_path(idnidx2, 9000, '18', 'Right') 

# lap3_path = r1+r2
# lap3_ids = ids1+ids2
# lap3_vs = vs1+vs2
# start_pose = r2[-1]

# # LAP 4
# start_ll = gput.lanelet_matching(start_pose)
# r1, idnidx1, ids1, vs1 = gput.get_straight_path(start_ll, 400, '59', 'Right')
# idnidx2 = gput.get_merged_point(idnidx1, diag_len, 2)
# r2, idnidx3, ids2, vs2 = gput.get_straight_path(idnidx2, 9000, '19', 'Right') 

# lap4_path = r1+r2
# lap4_ids = ids1+ids2
# lap4_vs = vs1+vs2
# start_pose = r2[-1]

# #LAP 5
# start_ll = gput.lanelet_matching(start_pose)
# r1, idnidx1, ids1, vs1 = gput.get_straight_path(start_ll, 900, '61', 'Right')
# r2, idnidx2, ids2, vs2 = gput.get_straight_path(idnidx1, 9000, '25', 'Right') 

# lap5_path = r1+r2
# lap5_ids = ids1+ids2
# lap5_vs = vs1+vs2
# start_pose = r2[-1]


# final_path = lap1_path+lap2_path+lap3_path+lap4_path+lap5_path
# final_ids = lap1_ids+lap2_ids+lap3_ids+lap4_ids+lap5_ids
# final_vs = lap1_vs+lap2_vs+lap3_vs+lap4_vs+lap5_vs

final_path = lap1_path
final_ids = lap1_ids
final_vs = lap1_vs

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

save_.to_csv('./paths/Solbat.csv', final_tr)
save_.to_txt('./ids/Solbat_id.txt', final_ids)

print("saved")
pr1_path_viz = gput.PreRound1Viz(final_path)

rate = rospy.Rate(0.5)
while not rospy.is_shutdown():
    pub_pr1_path.publish(pr1_path_viz)
    rate.sleep()
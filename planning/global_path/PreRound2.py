import rospy
import copy
import numpy as np
from visualization_msgs.msg import Marker

from libs.lanelet import LaneletMap, TileMap
from libs.micro_lanelet_graph import MicroLaneletGraph
import libs.gp_utils as gput
import libs.save_ as save_

rospy.init_node('PreRound2', anonymous=False)
pub_pr2_path = rospy.Publisher('/pr2_path', Marker, queue_size=1)

start_index = 1 # 0 : lane 2, 1 : lane 4
tile_size = 5
cut_dist = 15
precision = 1
diag_len = 25
lane_width = 3.25
max_vel = 80 / 3.6
sec_to_reach = 7
dist_to_reach_max_vel = int((0.5*(max_vel/sec_to_reach)*(sec_to_reach**2)))

lmap = LaneletMap('/home/inha/catkin_ws/src/mobinha-racing/map_lane/hd_map/KIAPI_Racing.json')
tmap = TileMap(lmap.lanelets, tile_size)
mlg = MicroLaneletGraph(lmap, cut_dist)
graph = mlg.graph


gput.lanelets = mlg.lanelets
gput.tiles = tmap.tiles
gput.tile_size = tile_size
gput.lane_width = lane_width

# Path Generation
start_poses = [(-7.514, -0.297),(0,0)]
start_pose = start_poses[start_index]
final_path = []
final_ids = []
final_vs = []

if start_index == 0:
    start_ll = gput.lanelet_matching(start_pose)
    r1, idnidx1, ids1, vs1 = gput.get_straight_path(start_ll, 7000, '23')
    idnidx2 = gput.get_merged_point(idnidx1, diag_len, 2)
    r2, idnidx3, ids2, vs2 = gput.get_straight_path(idnidx2, 6000, '74', 'Left')
    idnidx4 = gput.get_merged_point(idnidx3, diag_len, 2)
    r3, _, ids3, vs3 = gput.get_straight_path(idnidx4, 6000, '21')
    start_pose = r3[-1]
    final_path = r1+r2+r3
    final_ids = ids1+ids2+ids3
    final_vs = vs1+vs2+vs3
else:
    start_ll = gput.lanelet_matching(start_pose)
    r1, idnidx1, ids1, vs1 = gput.get_straight_path(start_ll, 7000, '22', 'Right')
    idnidx2 = gput.get_merged_point(idnidx1, diag_len, 1)
    r2, idnidx3, ids2,vs2 = gput.get_straight_path(idnidx2, 6000, '73')
    idnidx4 = gput.get_merged_point(idnidx3, diag_len, 2)
    r3, _, ids3,vs3 = gput.get_straight_path(idnidx4, 6000, '21')
    start_pose = r3[-1]
    final_path = r1+r2+r3
    final_ids = ids1+ids2+ids3
    final_vs = vs1+vs2+vs3

for i in range(3,6):
    start_ll = gput.lanelet_matching(start_pose)
    r1, idnidx1, ids1,vs1 = gput.get_straight_path(start_ll, 400, '64', 'Right')
    r2, idnidx2, ids2, vs2= gput.get_straight_path(idnidx1, 7000, '21', 'Right')
    lap_path = r1+r2
    lap_id = ids1+ids2
    lap_vs = vs1+vs2
    start_pose = r2[-1] 
    if i == 5:
        idnidx3 = gput.get_merged_point(idnidx2, diag_len, 2)
        r3, idnidx4, ids3, vs3 = gput.get_straight_path(idnidx3, 20, '27')
        lap_path = lap_path+r3
        lap_id = lap_id+ids3
        lap_vs = lap_vs+vs3
    final_path = final_path+lap_path
    final_ids = final_ids+lap_id
    final_vs = final_vs+lap_vs


final_tr = []
copy_final_path = copy.deepcopy(final_path)
copy_final_path.insert(0, final_path[0])
copy_final_path.append(final_path[-1])

#accel_p, vel_p, dist_p = gput.get_profiles(len(final_path), max_vel, sec_to_reach)
vel_p, accel_p, dist_p = gput.adjust_velocity_profile(final_vs)

s = 0

for i,f in enumerate(final_path):
    before_after_pts = [copy_final_path[i], copy_final_path[i+2]]
    lw_left, lw_right = gput.get_lane_width(final_ids[i])
    A, B, theta = gput.calc_norm_vec(before_after_pts)
    Rk = gput.calc_kappa(f, before_after_pts)
    #closest_index = np.argmin(np.abs(dist_p-s))
    vx = vel_p[i]
    ax = accel_p[i]

    final_tr.append([f[0], f[1], lw_right, lw_left, A, B, 0, s, theta, Rk, vx, ax])
    s += 1


save_.to_csv('./PreRound2b.csv', final_tr)
save_.to_txt('./PreRound2b_id.txt', final_ids)

print("saved")

# pr2_path = gput.smooth_interpolate(final_path, precision)
# pr2_path_viz = gput.PreRound2Viz(pr2_path)

# rate = rospy.Rate(5)
# while not rospy.is_shutdown():
#     pub_pr2_path.publish(pr2_path_viz)
#     rate.sleep()

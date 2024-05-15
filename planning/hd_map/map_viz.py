import rospy
from map_utils import *
from lanelet import LaneletMap
from micro_lanelet_graph import MicroLaneletGraph

rospy.init_node('Map', anonymous=False)

lmap = LaneletMap('./Pangyo.json')
graph = MicroLaneletGraph(lmap, 15).graph
lmap_viz = LaneletMapViz(lmap.lanelets, lmap.for_viz)
mlmap_viz = MicroLaneletGraphViz(lmap.lanelets, graph)
pub_lmap_viz = rospy.Publisher('/lmap', MarkerArray, queue_size=1)
pub_mlmap_viz = rospy.Publisher('/mlmap', MarkerArray, queue_size=1)

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    pub_lmap_viz.publish(lmap_viz)
    pub_mlmap_viz.publish(mlmap_viz)
    rate.sleep()
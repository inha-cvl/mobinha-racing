import rospy
import time
from hd_map.libs.map_utils import *
from hd_map.map import MAP

rospy.init_node('Map', anonymous=False)

map = MAP('KIAPI_Racing_Fast')
lmap_viz = map.lmap_viz
mlmap_viz = map.mlmap_viz

pub_lmap_viz = rospy.Publisher('/lmap', MarkerArray, queue_size=1)
pub_mlmap_viz = rospy.Publisher('/mlmap', MarkerArray, queue_size=1)

rate = rospy.Rate(0.5)
while not rospy.is_shutdown():
    pub_lmap_viz.publish(lmap_viz)
    pub_mlmap_viz.publish(mlmap_viz)
    rate.sleep()
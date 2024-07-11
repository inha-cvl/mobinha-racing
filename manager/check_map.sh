#!/bin/bash

cd ../ui/rviz
rosrun rviz rviz -d map_only.rviz &
cd ../../map_lane
python3 map_viz.py
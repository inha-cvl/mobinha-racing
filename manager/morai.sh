#!/bin/bash
rosparam load coordinates.yaml &
cd ../ui/
python3 ui.py & 
cd ../visualizer
python3 visualizer.py &
cd ../map_lane
python3 map_lane.py &
cd ../simulator/
python3 morai.py & 
cd ../drive_message/
python3 drive_message.py KIAPI_Racing & 
cd ../control/
python3 control.py &
cd ../planning/
python3 planning_frenet.py to_goal
#!/bin/bash
rosparam load coordinates.yaml &
cd ../ui/
python3 ui.py & 
cd ../visualizer
python3 visualizer.py &
cd ../drive_message/
python3 drive_message.py KCity &
sleep 2 
cd ../map_lane
python3 map_lane.py &
cd ../simulator/
python3 morai.py & 
cd ../control/
python3 control.py
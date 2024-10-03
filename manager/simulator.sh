#!/bin/bash
rosparam load coordinates.yaml &
cd ../ui/
python3 ui.py & 
cd ../visualizer
python3 visualizer.py &
cd ../drive_message/
python3 drive_message.py KIAPI_Racing_Fast &
sleep 3 
cd ../map_lane
python3 map_lane.py &
cd ../simulator
python3 object_simulator.py &
cd ../simulator/
python3 car_simulator.py & 
cd ../control/
python3 control.py &
cd ../planning/
python3 planning2.py
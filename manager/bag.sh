#!/bin/bash
rosparam load coordinates.yaml &
sleep 1
cd ../ui/
python3 ui.py &  
cd ../visualizer #0
python3 visualizer.py &
cd ../drive_message/ #0
python3 drive_message.py KIAPI_Racing_Fast &
cd ../perception/
python3 perception.py &
sleep 2 
cd ../map_lane
python3 map_lane.py &
cd ../simulator/
python3 car_simulator.py 
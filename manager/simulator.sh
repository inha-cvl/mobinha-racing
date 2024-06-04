#!/bin/bash

cd ../ui/
python3 ui.py & 
cd ../visualizer
python3 visualizer.py &
cd ../map_lane
python3 map_lane.py &
cd ../simulator
python3 object_simulator.py &
# cd ../simulator/
# python3 car_simulator.py & 
# cd ../drive_message/
# python3 drive_message.py KIAPI_Racing& 
cd ../control/
python3 control.py 
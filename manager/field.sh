#!/bin/bash

cd ../ui/
python3 ui.py & 
cd ../visualizer
python3 visualizer.py &
cd ../map_lane
python3 map_lane.py &
cd ../transmitter/
python3 transmitter.py & 
cd ../drive_message/
python3 drive_message.py Harbor& 
cd ../control/  
python3 control.py #& 
# cd ../planning/
# python3 planning.py
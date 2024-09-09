#!/bin/bash
rosparam load coordinates.yaml &
sleep 1
cd ../ui/
python3 ui.py & 
# cd ../localization/
# python3 localization.py &
cd ../visualizer
python3 visualizer.py &
cd ../drive_message/
python3 drive_message.py Solchan &
sleep 2 
cd ../map_lane
python3 map_lane.py &
# cd ../transmitter/
# python3 transmitter.py & 
# cd ../control/  
# python3 control.py &
# cd ../perception/
# python3 perception.py &
cd ../planning/
python3 planning.py 

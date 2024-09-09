#!/bin/bash
rosparam load coordinates_solchan.yaml &
sleep 1
cd ../ui/
python3 ui.py & 
cd ../localization/
python3 localization_new.py &
# python3 localization_sbg.py &
cd ../visualizer
python3 visualizer.py &
cd ../drive_message/
python3 drive_message.py Solchan &
sleep 2 
cd ../map_lane
python3 map_lane.py &
cd ../transmitter/
python3 transmitter.py & 
cd ../control/  
python3 control.py &
cd ../planning/
python3 planning.py &

cd ~/bag/
rosbag record /ADAS_DRV /CANInput /CANOutput /DetectionData /EgoActuator /LaneData /LaneLet /NavigationData /RadarObjectArray /SensorData /SystemStatus /UserInput /VehicleState /camera/image_color/compressed /control/target_actuator /global_path /lmap /map_lane/refine_obstacles /mobinha/perception/lidar/track_box /novatel/oem7/bestpos /novatel/oem7/inspva /sbg/ekf_nav /imu/nav_sat_fix /sbg/ekf_euler /planning/local_path /planning/target_object /rosout /rosout_agg /tf /tf_static
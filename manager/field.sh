#!/bin/bash
rosparam load coordinates.yaml &
sleep 1
cd ../ui/
python3 ui.py & 
cd ../localization/
python3 localization_new.py &
cd ../visualizer
python3 visualizer.py &
cd ../drive_message/
python3 drive_message.py KIAPI_Racing_Fast &
sleep 2 
cd ../map_lane
python3 map_lane.py &
cd ../simulator
python3 object_simulator.py &
cd ../transmitter/
python3 transmitter.py & 
cd ../control/  
python3 control.py &
cd ../planning/
python3 planning2.py &

cd ~/bag/
# rosbag record /ADAS_DRV /CANInput /CANOutput /DetectionData /EgoActuator /LaneData /LaneLet /NavigationData /RadarObjectArray /SensorData /SystemStatus /UserInput /VehicleState /camera/image_color/compressed /control/target_actuator /global_path /lmap /map_lane/refine_obstacles /mobinha/perception/lidar/track_box /novatel/oem7/bestpos /novatel/oem7/inspva /sbg/ekf_nav /imu/nav_sat_fix /sbg/ekf_euler /planning/local_path /planning/target_object /rosout /rosout_agg /tf /tf_static
rosbag record /ADAS_DRV /CANInput /CANOutput /DetectionData /EgoActuator /LaneData /LaneLet /NavigationData /RadarObjectArray /SensorData /SystemStatus /UserInput /VehicleState /camera/image_color/compressed /control/target_actuator /global_path /lmap /map_lane/refine_obstacles /mobinha/perception/lidar/track_box /ublox/navpvt /ublox/navatt /best/pose /planning/local_path /planning/target_object /rosout /rosout_agg /tf /tf_static
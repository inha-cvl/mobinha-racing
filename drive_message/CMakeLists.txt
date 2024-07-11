cmake_minimum_required(VERSION 3.0.2)
project(drive_msgs)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  Actuator.msg
  ObjectInfo.msg
  DetectionData.msg
  NavigationData.msg
  VehicleState.msg
  CANInput.msg
  LaneData.msg
  SensorData.msg
  CANOutput.msg
  LaneLet.msg
  UserInput.msg
  SystemStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
 LIBRARIES drive_msgs
 CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set_source_files_properties( PROPERTIES LANGUAGE CXX)
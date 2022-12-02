cmake_minimum_required(VERSION 3.0.2)
project(gnss_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  ublox_msgs
  message_generation
)


add_message_files(
  FILES
  raw.msg
  gnss_raw.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
 std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge sensor_msgs ublox_msgs message_runtime
)

include_directories(
  include()
  ${catkin_INCLUDE_DIRS}
)

add_executable(gnss_raw src/gnss_raw.cpp)
target_link_libraries(gnss_raw ${catkin_LIBRARIES})
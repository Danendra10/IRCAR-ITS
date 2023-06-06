cmake_minimum_required(VERSION 3.0.2)
project(master)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  nav_msgs
  utils
  motion
  geometry_msgs
  msg_collection
)

find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(master src/master.cc)
target_link_libraries(master ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} entity_lib motion_lib)
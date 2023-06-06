cmake_minimum_required(VERSION 3.0.2)
project(motion)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  utils
  msg_collection
)

catkin_package(
 INCLUDE_DIRS include

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(motion_lib
  src/motion.cpp
)

target_link_libraries(motion_lib ${catkin_LIBRARIES} entity_lib math_lib pid_lib potential_field_lib)
cmake_minimum_required(VERSION 3.0.2)
project(motion)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  utils
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES motion
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/motion.cpp
)

target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} entity_lib math_lib pid_lib potential_field_lib)
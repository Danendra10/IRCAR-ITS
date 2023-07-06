cmake_minimum_required(VERSION 3.0.2)
project(vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  nav_msgs
  utils
  msg_collection
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



add_executable(vision_cpp src/vision.cpp)
target_link_libraries(vision_cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} entity_lib math_lib logger_lib)

add_executable(lidar src/lidar.cpp)
target_link_libraries(lidar ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} entity_lib)

add_executable(anderdevelop_cpp src/anderdevelop.cpp)
target_link_libraries(anderdevelop_cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} entity_lib math_lib logger_lib)

<<<<<<< HEAD

=======
>>>>>>> master
add_executable(road_sign_detector src/road_sign_detector.cpp)
target_link_libraries(road_sign_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} entity_lib math_lib logger_lib)
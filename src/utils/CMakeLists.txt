cmake_minimum_required(VERSION 3.0.2)
project(utils)

add_compile_options(-std=c++11)

catkin_package(
  INCLUDE_DIRS include  
)

find_package(OpenCV REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(entity_lib
  src/entity.cc
)
target_link_libraries(entity_lib ${catkin_LIBRARIES})

add_library(math_lib
  src/math.cc
)
target_link_libraries(math_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
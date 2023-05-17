cmake_minimum_required(VERSION 3.0.2)
project(utils)

add_compile_options(-std=c++11)

catkin_package(
  INCLUDE_DIRS include  
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(entity_lib
  src/entity.cc
)
target_link_libraries(entity_lib ${catkin_LIBRARIES})
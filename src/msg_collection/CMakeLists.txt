cmake_minimum_required(VERSION 3.0.2)
project(msg_collection)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
    FILES
    PointArray.msg
    Obstacles.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  )
  

catkin_package(
#  INCLUDE_DIRS include
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

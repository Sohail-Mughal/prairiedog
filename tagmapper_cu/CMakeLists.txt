
cmake_minimum_required(VERSION 2.8.3)
project(tagmapper_cu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  stargazer_cu
  localization_cu
  irobot_create_rustic
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  stargazer_cu
  localization_cu
  irobot_create_rustic
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tagmapper_cu  
  src/tagmapper_cu.cpp
)

target_link_libraries(tagmapper_cu
  ${catkin_LIBRARIES}
)

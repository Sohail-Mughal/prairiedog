cmake_minimum_required(VERSION 2.8.3)
project(mapper_cu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  hokuyo_listener_cu
  tf
)
catkin_package(
  CATKIN_DEPENDS 
    roscpp
    nav_msgs
    geometry_msgs
    sensor_msgs
    hokuyo_listener_cu
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mapper_cu  
  src/mapper_cu.cpp)

target_link_libraries(mapper_cu
  ${catkin_LIBRARIES}
)


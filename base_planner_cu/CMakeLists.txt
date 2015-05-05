cmake_minimum_required(VERSION 2.8.3)
project(base_planner_cu)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  mapper_cu
  localization_cu
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    nav_msgs
    geometry_msgs
    sensor_msgs
    mapper_cu
    localization_cu
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_planner_cu 
    src/base_planner_cu.cpp
)

target_link_libraries(base_planner_cu
  ${catkin_LIBRARIES}
)

cmake_minimum_required(VERSION 2.8.3)
project(prairiedog)

find_package(catkin REQUIRED COMPONENTS
  common_msgs
  image_transport_plugins
  laser_drivers
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES prairiedog
  CATKIN_DEPENDS 
    common_msgs 
    image_transport_plugins 
    laser_drivers
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

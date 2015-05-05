cmake_minimum_required(VERSION 2.8.3)
project(localization_cu)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    nav_msgs
    tf
    message_generation
)


add_service_files(
  FILES
    GetPose.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs    
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    geometry_msgs
    nav_msgs
    tf
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(localization_cu 
  src/localization_cu.cpp
)

target_link_libraries(localization_cu
  ${catkin_LIBRARIES}
)

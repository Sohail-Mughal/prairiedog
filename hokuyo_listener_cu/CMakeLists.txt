cmake_minimum_required(VERSION 2.8.3)
project(hokuyo_listener_cu)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    hokuyo_node
    tf
    message_generation
)


add_message_files(
  FILES
    PointCloudWithOrigin.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    geometry_msgs
    sensor_msgs
    hokuyo_node
    tf
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hokuyo_listener_cu 
  src/hokuyo_listener_cu.cpp
)

target_link_libraries(hokuyo_listener_cu
  ${catkin_LIBRARIES}
)

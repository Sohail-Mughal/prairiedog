cmake_minimum_required(VERSION 2.8.3)
project(stargazer_cu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)  

add_message_files(
  FILES
    Pose2DTagged.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs    
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    std_msgs
    geometry_msgs
    message_runtime
)

include_directories(
  include/tinyxml
  ${catkin_INCLUDE_DIRS}
  src
)

add_library(stargazer_cu_lib 
  include/tinyxml/tinyxml.cpp
  include/tinyxml/tinyxmlerror.cpp
  include/tinyxml/tinyxmlparser.cpp
  include/tinyxml/tinystr.cpp
  src/serial_communication.cpp
)

add_executable(stargazer_cu
  src/stargazer_cu.cpp
)

target_link_libraries(stargazer_cu
  ${catkin_LIBRARIES}
  stargazer_cu_lib
)

add_executable(test_cu
  src/test.cpp
)

target_link_libraries(test_cu
  ${catkin_LIBRARIES}
  stargazer_cu_lib
)

#common commands for building c++ executables and libraries
#rosbuild_add_library(hagisonic src/XMLite.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


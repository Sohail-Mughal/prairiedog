cmake_minimum_required(VERSION 2.8.3)
project(irobot_create_rustic)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    rospy
    message_generation
)


add_message_files(
  FILES
    Bumper.msg
    Position2D.msg
    Power.msg
    Sound.msg
    Speeds.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs    
)

catkin_package(
  CATKIN_DEPENDS 
    roscpp
    std_msgs
    rospy
    message_runtime
)

add_library(irobot_create_controller src/irobot_create_controller/controller.cpp)
add_library(create_comms src/irobot_create_driver/create_comms.c)

add_executable(driver src/irobot_create_driver/driver.cpp)
target_link_libraries(driver create_comms)


add_executable(test_1 src/irobot_create_examples/test_1.cpp)
target_link_libraries(test_1 irobot_create_controller)

add_executable(listener_test src/irobot_create_examples/listener_test.cpp)
target_link_libraries(listener_test irobot_create_controller)

add_executable(mario_song src/irobot_create_examples/mario_song.cpp)
target_link_libraries(mario_song irobot_create_controller)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${wxWidgets_INCLUDE_DIRS}
)

add_library(irobot_create_gui src/irobot_create_gui/slider_2d.cpp)
target_link_libraries(irobot_create_gui ${wxWidgets_LIBRARIES})

add_executable(controller_gui src/irobot_create_gui/controller_gui.cpp)
target_link_libraries(controller_gui irobot_create_gui irobot_create_controller)


target_link_libraries(irobot_create_rustic
  ${catkin_LIBRARIES}
)
     

#add_subdirectory(src)


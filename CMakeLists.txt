cmake_minimum_required(VERSION 2.8.3)
project(twizy)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)

## Declare ROS messages and services
add_message_files(
FILES
car_control.msg
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
DEPENDS
roscpp
rospy
message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(control_node
scripts/control_listener.cpp
scripts/CANComunication.cpp
)

target_link_libraries(control_node
${catkin_LIBRARIES}
canlib
)

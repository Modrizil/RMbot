cmake_minimum_required(VERSION 3.0.2)
project(RMbot_bringup)

include_directories(include)
include_directories(src)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  message_generation
  std_msgs
  serial
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES RMbot_bringup
 CATKIN_DEPENDS roscpp rospy message_runtime serial
 DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_NAME}/include
)

## Declare a C++ library
add_library(chassis_tfpub
  src/chassis_tfpub.cpp
)
add_library(com_controller
  src/com_controller.cpp
)

add_executable(RMbot_chassis_node 
  src/RMbot_chassis.cpp
  src/chassis_tfpub.cpp
  src/com_controller.cpp
)

target_link_libraries(RMbot_chassis_node
  ${catkin_LIBRARIES}
)
# target_include_directories(RMbot_chassis_node
#   ${catkin_INCLUDE_DIRS}
# )

add_executable(rmbot_teleop
  src/rmbot_teleop.cpp
)

target_link_libraries(rmbot_teleop
  ${catkin_LIBRARIES}
)

add_executable(self_circle_test
  src/self_circle_test.cpp
)

target_link_libraries(self_circle_test
  ${catkin_LIBRARIES}
)
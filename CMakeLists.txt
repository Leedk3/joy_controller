cmake_minimum_required(VERSION 2.8.3)
project(etri_joy_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  ackermann_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

set( sources
  src/joy_stick_control.cpp
)

add_executable(${PROJECT_NAME}_node ${sources})

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)



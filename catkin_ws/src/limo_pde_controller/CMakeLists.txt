cmake_minimum_required(VERSION 3.0.2)
project(limo_pde_controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
)

## Declare a catkin package
catkin_package()

## Build includes and libraries
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare executables
add_executable(limo_pde_circle_controller src/limo_pde_circle_controller.cpp)
target_link_libraries(limo_pde_circle_controller ${catkin_LIBRARIES})
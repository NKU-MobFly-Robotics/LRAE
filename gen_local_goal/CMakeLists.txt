cmake_minimum_required(VERSION 3.0.2)
project(gen_local_goal)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pluginlib
  roscpp
  std_msgs
  tf
  tf2
)

# Eigen3
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS    
  nav_msgs  
  roscpp
  tf 
  tf2
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_executable(gen_local_goal_node src/gen_local_goal_node.cpp src/gen_local_goal.cpp)
add_dependencies(gen_local_goal_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(gen_local_goal_node ${catkin_LIBRARIES} )

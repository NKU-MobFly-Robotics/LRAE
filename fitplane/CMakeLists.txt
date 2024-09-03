cmake_minimum_required(VERSION 3.0.2)
project(fitplane)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  # rviz_visual_tools
  std_msgs
  message_generation
  tf
)
add_message_files(
  FILES
  Plane.msg
  PlaneMap.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

add_executable(Traversibility_mapping
    src/FitPlane.cpp
    src/World.cpp
    src/plane.cpp)

target_link_libraries(Traversibility_mapping 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)

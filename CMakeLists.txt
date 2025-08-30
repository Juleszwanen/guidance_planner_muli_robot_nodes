cmake_minimum_required(VERSION 3.8)
project(guidance_planner_multi_robot_nodes)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(DEPENDENCIES
  roscpp
  mpc_planner
  mpc_planner_msgs
  mpc_planner_modules
  mpc_planner_solver
  mpc_planner_util
  mpc_planner_types
  ros_tools
  nav_msgs
  dynamic_reconfigure
  tf2_ros
)

find_package(catkin REQUIRED COMPONENTS
  ${DEPENDENCIES}
)




catkin_package(
   CATKIN_DEPENDS ${DEPENDENCIES}
   INCLUDE_DIRS include/${PROJECT_NAME} 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Import, so that the macros are correctly applied and the correct header files are included!
add_definitions(-DMPC_PLANNER_ROS)

add_executable(central_aggregator src/central_aggregator_node.cpp)
add_dependencies(central_aggregator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(central_aggregator ${catkin_LIBRARIES})






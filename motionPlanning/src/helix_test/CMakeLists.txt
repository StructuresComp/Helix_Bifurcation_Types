cmake_minimum_required(VERSION 3.0.2)
project(helix_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  descartes_core
  descartes_planner
  descartes_moveit
  descartes_utilities
  moveit_ros_planning
  moveit_msgs
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_core
  eigen_conversions
  moveit_visual_tools
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS
     include
   LIBRARIES
     ${PROJECT_NAME}
  CATKIN_DEPENDS
     descartes_core
     descartes_planner
     descartes_moveit
     descartes_utilities
     moveit_core
     moveit_msgs
     moveit_ros_planning_interface
     moveit_ros_planning
     moveit_ros_move_group
     moveit_visual_tools
     eigen_conversions
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

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/demo_application.cpp
   src/tasks/init_ros.cpp
   src/tasks/init_descartes.cpp
   src/tasks/move_home.cpp
   src/tasks/generate_trajectory.cpp
   src/tasks/plan_path.cpp
   src/tasks/run_path.cpp
   src/tasks/load_parameters.cpp
   src/computePath.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## Declare a cpp executable
add_executable(helix_test_node src/helix_test_node.cpp)
target_link_libraries(helix_test_node ${PROJECT_NAME} ${catkin_LIBRARIES})

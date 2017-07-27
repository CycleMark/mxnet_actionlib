cmake_minimum_required(VERSION 2.8.3)
project(mxnet_actionlib)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  ecl_exceptions ecl_threads ecl_time 
          ecl_build
        ecl_threads
        ecl_geometry
        ecl_linear_algebra
        kobuki_msgs
        kdl_conversions
        message_filters
)

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   AutoDocking.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
 )

catkin_package(
  CATKIN_DEPENDS actionlib_msgs ecl_exceptions ecl_threads ecl_time
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy message_filters actionlib kdl_conversions
                  ecl_threads ecl_geometry ecl_linear_algebra ecl_core kobuki_msgs)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mxnet_actionlib
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)
#add_library(dock_drive
#   src/dock_drive.cpp
#)

add_executable(auto_docking_server src/auto_docking_server.cpp src/dock_drive.cpp src/dock_drive_states.cpp)

target_link_libraries( 
  auto_docking_server
  ${catkin_LIBRARIES}
)

add_executable(auto_docking_client src/auto_docking_client.cpp)

target_link_libraries( 
  auto_docking_client
  ${catkin_LIBRARIES}
)
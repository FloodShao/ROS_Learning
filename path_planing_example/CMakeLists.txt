cmake_minimum_required(VERSION 2.8.3)
project(path_planing_example)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)
find_package(OpenCV REQUIRED)
find_package(Armadillo REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS
  message_runtime
  roscpp std_msgs
  geometry_msgs
  sensor_msgs
)

###########
## Build ##
###########

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${openCV_INCLUDE_DIRS}
)

## Declare a C++ executable:
add_executable(camera_operations src/camera_operations.cpp)
target_link_libraries(camera_operations ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(navigator src/navigator.cpp)
target_link_libraries(navigator ${catkin_LIBRARIES})

add_executable(path_planner src/path_planner.cpp)
target_link_libraries(path_planner ${catkin_LIBRARIES})

add_executable(range_finder src/range_finder.cpp)
target_link_libraries(range_finder ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)
project(fflocking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  std_msgs
  message_generation
  cmake_modules
  dynamic_reconfigure
)

find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
# find
# )
add_message_files(
   FILES
   Point2D.msg
)

add_service_files(
  FILES
  Planning_qfy.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


generate_dynamic_reconfigure_options(
  cfg/FlockingAlgo.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES flocking_algo planning_ros
  CATKIN_DEPENDS message_runtime
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(flocking_algo
  src/flocking_algo.cpp
  include/fflocking/flocking_algo.hpp
)
target_link_libraries(flocking_algo
  ${catkin_LIBRARIES}
)
add_dependencies(flocking_algo ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  fflocking_generate_messages_cpp)

add_library(planning_ros
  src/planning_ros.cpp
  include/fflocking/planning_ros.hpp
)
target_link_libraries(planning_ros
  ${catkin_LIBRARIES}
)
add_dependencies(planning_ros ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  fflocking_generate_messages_cpp
  fflocking_gencfg
  )

# Add executable

add_executable(flocking_node
  src/flocking_node.cpp)
target_link_libraries(flocking_node
  ${catkin_LIBRARIES}
  planning_ros
  flocking_algo
)
add_dependencies(flocking_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
  fflocking_generate_messages_cpp
  )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/fflocking_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
  )

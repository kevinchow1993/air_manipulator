cmake_minimum_required(VERSION 2.8.3)
project(uav_body_convert)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  message_generation
  geometry_msgs
  mavros_msgs
)

set(EIGEN3_DIR "/home/kevin/catkin_ware/src/am_controller")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


generate_messages(DEPENDENCIES std_msgs geometry_msgs mavros_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)


include_directories(include
${catkin_INCLUDE_DIRS}
)



add_executable(uav_body_convert src/uav_body_convert.cpp)
target_link_libraries(uav_body_convert ${catkin_LIBRARIES})
add_dependencies(uav_body_convert uav_body_convert_generate_messages_cpp)




install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

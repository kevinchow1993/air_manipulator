cmake_minimum_required(VERSION 2.8.3)
project(ffcalib)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  tf
  std_msgs
  gazebo_msgs
  message_generation
  geometry_msgs
  nav_msgs
  mavros_msgs
  cmake_modules
)
set(OpenCV_DIR "/home/kevin/catkin_al/src/monocular_servo/ffcalibration")
find_package( OpenCV REQUIRED )

set(EIGEN3_DIR "/home/kevin/catkin_al/src/am_controller")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

#add_definitions(${EIGEN_DEFINITIONS})

#add_message_files(FILES ikMsg.msg control_state.msg)
#add_service_files(FILES ikSrv.srv)
#add_service_files(FILES graper.srv)

#generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)


include_directories(include
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

add_executable(ffcalib src/ffcalib.cpp)
target_link_libraries(ffcalib ${catkin_LIBRARIES})
add_dependencies(ffcalib ffcalib_generate_messages_cpp)




install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

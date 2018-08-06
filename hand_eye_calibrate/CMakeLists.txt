cmake_minimum_required(VERSION 2.8.3)
project(hand_eye_calibrate)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)



set(EIGEN3_DIR "/home/kevin/catkin_ware/src/am_controller")
find_package(Eigen3 REQUIRED)
include_directories(
  include ${catkin_INCLUDE_DIRS}${EIGEN3_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  geometry_msgs
  rospy
  std_msgs
  cmake_modules
  cv_bridge
  tf
  apriltags
  image_transport
  eigen_conversions
  tf_conversions
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime
)


include_directories(
  include ${catkin_INCLUDE_DIRS}

)

include_directories(/home/kevin/caktin_ware/devel/include)



add_executable(hand_eye_cal_sub
  src/hand_eye_cal_sub.cpp src/serial.cpp src/timeout.cpp
)
add_dependencies(hand_eye_cal_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hand_eye_cal_sub
  ${catkin_LIBRARIES}
)

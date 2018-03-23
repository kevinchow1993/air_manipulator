cmake_minimum_required(VERSION 2.8.3)
project(camera_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  gazebo_msgs
  message_generation
  geometry_msgs
  nav_msgs
  mavros_msgs
)

set(EIGEN3_DIR "/home/kevin/catkin_ware/src/am_controller")
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_message_files(FILES CameraPos.msg)
add_service_files(FILES ServoPos.srv)
add_service_files(FILES TagLost.srv)
add_service_files(FILES Grasping.srv)
add_service_files(FILES set_interest_ID.srv)
add_message_files(FILES World_pose.msg do_trajectory_task.msg)
add_service_files(FILES rc_state.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)


include_directories(include
${catkin_INCLUDE_DIRS}
)

add_executable(camera_servo src/camera_servo.cpp src/serial.cpp src/timeout.cpp)
target_link_libraries(camera_servo ${catkin_LIBRARIES})
add_dependencies(camera_servo camera_control_generate_messages_cpp)

add_executable(target_tracker src/target_tracker.cpp src/serial.cpp src/timeout.cpp)
target_link_libraries(target_tracker ${catkin_LIBRARIES})
add_dependencies(target_tracker camera_control_generate_messages_cpp)

add_executable(target_endeffector_track src/target_endeffector_track.cpp src/serial.cpp src/timeout.cpp)
target_link_libraries(target_endeffector_track ${catkin_LIBRARIES})
add_dependencies(target_endeffector_track camera_control_generate_messages_cpp)
#！！！
add_executable(target_path_track src/target_path_track.cpp src/serial.cpp src/timeout.cpp)
target_link_libraries(target_path_track ${catkin_LIBRARIES})
add_dependencies(target_path_track camera_control_generate_messages_cpp)

add_executable(global_control src/global_control.cpp)
target_link_libraries(global_control ${catkin_LIBRARIES})
add_dependencies(global_control camera_control_generate_messages_cpp)

add_executable(rc_state src/rc_state.cpp)
target_link_libraries(rc_state ${catkin_LIBRARIES})
add_dependencies(rc_state camera_control_generate_messages_cpp)
##！！！
add_executable(global_rc_path_plan src/global_rc_path_plan.cpp)
target_link_libraries(global_rc_path_plan ${catkin_LIBRARIES})
add_dependencies(global_rc_path_plan camera_control_generate_messages_cpp)


install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cmake_minimum_required(VERSION 2.8.3)
project(am_controller)

# OpenMP
message(STATUS "Checking for OpenMP support")
find_package(OpenMP)
if (OPENMP_FOUND)
  message(STATUS "OpenMP supported. Adding flags ${OpenMP_CXX_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  add_definitions(-DUSE_OPENMP)
else (OPENMP_FOUND)
  message(STATUS "OpenMP not supported")
endif (OPENMP_FOUND)


set(NSGA2_SOURCE_FILES
  src/nsga2/global.cpp
  src/nsga2/random.cpp
  src/nsga2/NSGA2.cpp
  #src/rand.c # should change to another random generator
  )




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
  genmsg
  actionlib_msgs
  actionlib
)

set(EIGEN3_DIR "/home/kevin/caktin_ware/src/am_controller")
find_package(Eigen3 REQUIRED)
include_directories(
  include ${catkin_INCLUDE_DIRS}${EIGEN3_INCLUDE_DIRS})
add_definitions(${EIGEN3_DEFINITIONS})
add_message_files(FILES servoset_msg.msg Mat_Tba.msg success_flag.msg)
add_service_files(FILES servoset_srv.srv)
add_action_files(DIRECTORY action FILES trajectory_para.action)

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)


include_directories(include
${catkin_INCLUDE_DIRS}
)

add_executable(serial_ros_flx src/serial_ros_flx.cpp src/Kin.cpp src/serial.cpp src/timeout.cpp)
target_link_libraries(serial_ros_flx ${catkin_LIBRARIES})
add_dependencies(serial_ros_flx am_controller_generate_messages_cpp)

add_executable(servo_seter src/servo_seter.cpp)
target_link_libraries(servo_seter ${catkin_LIBRARIES})
add_dependencies(servo_seter am_controller_generate_messages_cpp)

add_executable(trajectory_planner src/trajectory_planner.cpp src/Kin.cpp ${NSGA2_SOURCE_FILES})
target_link_libraries(trajectory_planner ${catkin_LIBRARIES})
add_dependencies(trajectory_planner am_controller_generate_messages_cpp)


add_executable(trajectory_controller src/trajectory_controller.cpp src/Kin.cpp)
target_link_libraries(trajectory_controller ${catkin_LIBRARIES})
add_dependencies(trajectory_controller am_controller_generate_messages_cpp)

add_executable(trajectory_planner_test src/trajectory_planner_test.cpp src/Kin.cpp ${NSGA2_SOURCE_FILES})
target_link_libraries(trajectory_planner_test ${catkin_LIBRARIES})
add_dependencies(trajectory_planner_test am_controller_generate_messages_cpp)

add_executable(workspace_test src/workspace_test.cpp src/Kin.cpp)
target_link_libraries(workspace_test ${catkin_LIBRARIES})
add_dependencies(workspace_test am_controller_generate_messages_cpp)


add_executable(trajectory_task_flag_puber src/trajectory_task_flag_puber.cpp)
target_link_libraries(trajectory_task_flag_puber ${catkin_LIBRARIES})
add_dependencies(trajectory_task_flag_puber am_controller_generate_messages_cpp)


install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


add_executable(servo_calibriation_pub
  src/servo_calibriation_pub.cpp
)
add_dependencies(servo_calibriation_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(servo_calibriation_pub
  ${catkin_LIBRARIES}
)
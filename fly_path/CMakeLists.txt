cmake_minimum_required(VERSION 2.8.3)
project(fly_path_flx)

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




generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs mavros_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)


include_directories(include
${catkin_INCLUDE_DIRS}
)



add_executable(fly_path src/fly_path.cpp)
target_link_libraries(fly_path ${catkin_LIBRARIES})
add_dependencies(fly_path fly_path_flx_generate_messages_cpp)


add_executable(Path_line_test src/Path_line_test.cpp)
target_link_libraries(Path_line_test ${catkin_LIBRARIES})
add_dependencies(Path_line_test fly_path_flx_generate_messages_cpp)

add_executable(path_rc_test src/path_rc_test.cpp)
target_link_libraries(path_rc_test ${catkin_LIBRARIES})
add_dependencies(path_rc_test fly_path_flx_generate_messages_cpp)


install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

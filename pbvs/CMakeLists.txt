cmake_minimum_required(VERSION 2.8.3)
add_definitions(-std=c++11)
project(pbvs)



set(EIGEN3_DIR "/home/kevin/catkin_ware/src/am_controller")
find_package(Eigen3 REQUIRED)
include_directories(
  include ${catkin_INCLUDE_DIRS}${EIGEN3_INCLUDE_DIRS})

find_package(VISP REQUIRED visp_core visp_io visp_vs )
find_package(OpenCV REQUIRED)
find_package(orocos_kdl REQUIRED)
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

  tf_conversions
  eigen_conversions
  kdl_conversions
  nav_msgs
)

#add_message_files(FILES
#  Point2D.msg
#  apriltag_info.msg
#  AprilTagDetection.msg
 # AprilTagDetectionArray.msg
#)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  DEPENDS kindr
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS}

)
link_directories(${orocos_kdl_LIBRARY_DIRS})
include_directories(${VISP_INCLUDE_DIRS})
include_directories(/usr/local/kindr)
include_directories(/home/kevin/caktin_ware/devel/include)





add_executable(pbvs_node_sub
  src/pbvs_node_sub.cpp 
)
add_dependencies(pbvs_node_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbvs_node_sub
  ${catkin_LIBRARIES} ${VISP_LIBRARIES}
)

add_executable(eigen_test_sub
  src/eigen_test_sub.cpp
)
add_dependencies(eigen_test_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(eigen_test_sub
  ${catkin_LIBRARIES} 
)

add_executable(pbvs_node
  src/pbvs_node.cpp src/serial.cpp src/timeout.cpp
)
add_dependencies(pbvs_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pbvs_node
  ${catkin_LIBRARIES}
)


add_executable(uav_speed
  src/uav_speed.cpp
)
add_dependencies(uav_speed ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uav_speed
  ${catkin_LIBRARIES}
)

# add_executable(kdl_trajectory_sub
#   src/kdl_trajectory_sub.cpp
# )
# add_dependencies(kdl_trajectory_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(kdl_trajectory_sub
#   ${catkin_LIBRARIES} ${orocos_kdl_LIBRARYS}
# )

add_executable(mani_tra_gen_sub
  src/mani_tra_gen_sub.cpp
)
add_dependencies(mani_tra_gen_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mani_tra_gen_sub
  ${catkin_LIBRARIES}
)

add_executable(dynamic_grasp_sub
  src/dynamic_grasp_sub.cpp
)
add_dependencies(dynamic_grasp_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamic_grasp_sub
  ${catkin_LIBRARIES}
)
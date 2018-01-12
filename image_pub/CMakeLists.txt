cmake_minimum_required(VERSION 2.8.3)
project(image_pub)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
	image_transport
	roscpp
	sensor_msgs
	cv_bridge
  message_generation
  std_msgs
  cmake_modules
   OpenCV
)

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_BUILD_TYPE Release)

catkin_package(
	CATKIN_DEPENDS 	image_transport 
			roscpp sensor_msgs geometry_msgs cv_bridge message_runtime  std_msgs
)

add_executable(image_pub src/image_pub.cpp)
target_link_libraries(image_pub ${catkin_LIBRARIES})
add_dependencies(image_pub  ${catkin_EXPORTED_TARGETS})

add_executable(stereo_image_pub src/stereo_image_pub.cpp)
target_link_libraries(stereo_image_pub ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(stereo_image_pub ${catkin_EXPORTED_TARGETS})


add_executable(zed_image_pub src/zed_image.cpp)
target_link_libraries(zed_image_pub ${catkin_LIBRARIES})
add_dependencies(zed_image_pub  ${catkin_EXPORTED_TARGETS})

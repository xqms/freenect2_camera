
cmake_minimum_required(VERSION 2.8)

project(freenect2_camera)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	nodelet
	sensor_msgs
	image_transport
	camera_info_manager
)

find_path(LIBFREENECT2_INCLUDE_DIR freenect2.h)
find_library(LIBFREENECT2_LIBRARY freenect2)

include_directories(
	${catkin_INCLUDE_DIRS}
	${LIBFREENECT2_INCLUDE_DIR}
)

catkin_package()

add_library(freenect2_camera
	src/freenect2_camera.cpp
)
target_link_libraries(freenect2_camera
	${catkin_LIBRARIES}
	${LIBFREENECT2_LIBRARY}
)

cmake_minimum_required(VERSION 2.8.3)
project(occam_node)
find_package(catkin 
	REQUIRED COMPONENTS 
		image_transport 
		roscpp 
		std_msgs 
		sensor_msgs 
                camera_info_manager
                occam_sdk)


set(ROS_BUILD_TYPE Release)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_LIBUSB libusb-1.0)
include_directories(${PC_LIBUSB_INCLUDE_DIRS})
link_directories(${PC_LIBUSB_LIBDIR})

find_package(OpenCV REQUIRED)

catkin_package(
	INCLUDE_DIRS
	CATKIN_DEPENDS
		image_transport 
		roscpp 
		std_msgs 
		sensor_msgs
                occam_sdk
        camera_info_manager
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -Wno-format-extra-args")

include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(ros_indigosdk_node nodes/ros_indigosdk_node.cpp)
target_link_libraries(ros_indigosdk_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(rectify nodes/rectify.cpp)
target_link_libraries(rectify ${catkin_LIBRARIES})

add_executable(cloud nodes/cloud.cpp)
target_link_libraries(cloud ${catkin_LIBRARIES})

add_executable(stitch nodes/stitch.cpp)
target_link_libraries(stitch ${catkin_LIBRARIES})

add_executable(imgproc nodes/imgproc.cpp)
target_link_libraries(imgproc ${catkin_LIBRARIES})

install(TARGETS ros_indigosdk_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

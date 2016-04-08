cmake_minimum_required(VERSION 2.8.3)
project(occam_node)

# _______________SET________________________

set(ROS_BUILD_TYPE Release)

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x -Wno-format-extra-args")


# _______________PACKAGES____________________

find_package(catkin 
	REQUIRED COMPONENTS 
		image_transport 
		roscpp 
		std_msgs 
		sensor_msgs 
                cv_bridge
        camera_info_manager
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_LIBUSB libusb-1.0)
link_directories(${PC_LIBUSB_LIBDIR})

catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS
                image_transport
                roscpp
                std_msgs
                sensor_msgs
                cv_bridge
        camera_info_manager
)

# _______________INDIGO SDK____________________


set(INDIGOSDK_PATH "/home/jonas/git/occam/occam_sdk")   # <----- ADAPT ----------------------------------

set(INDIGOSDK_INCLUDE_DIR "${INDIGOSDK_PATH}/include")
find_library(INDIGO_LIB NAMES indigo HINTS "${INDIGOSDK_PATH}/release/bin")

# ___________ INCLUDES ______________

include_directories(
        include
	${catkin_INCLUDE_DIRS}
        ${INDIGOSDK_INCLUDE_DIR}
        ${PC_LIBUSB_INCLUDE_DIRS}
)

# ___________ EXECUTABLES ______________

add_executable(ros_indigosdk_node
    nodes/ros_indigosdk_node.cpp
)
target_link_libraries(ros_indigosdk_node
    ${INDIGO_LIB}
    ${catkin_LIBRARIES}
)

add_executable(publishOccamData
    nodes/publishOccamData.cpp
)
target_link_libraries(publishOccamData
    ${INDIGO_LIB}
    ${catkin_LIBRARIES}
)

# ___________ INSTALL ______________

install(TARGETS ros_indigosdk_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

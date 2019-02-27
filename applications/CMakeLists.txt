cmake_minimum_required(VERSION 2.6)

project(RGBD_RTK)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set( CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-std=c++11)

message(STATUS ">>> applications subdirectory")


add_executable(optical_flow_visual_odometry_test optical_flow_visual_odometry_test.cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

find_library(RGBD_RTK_IO NAMES rgbd_rtk_io)
find_library(RGBD_RTK_TRACKING NAMES rgbd_rtk_tracking)
find_library(RGBD_RTK_COMMON NAMES rgbd_rtk_common)
find_library(RGBD_RTK_VISUALIZATION NAMES rgbd_rtk_visualization)
find_library(RGBD_RTK_MOTION_ESTIMATION NAMES rgbd_rtk_motion_estimation)
find_library(RGBD_RTK_VISUAL_ODOMETRY NAMES rgbd_rtk_visual_odometry)


target_link_libraries(optical_flow_visual_odometry_test ${PCL_LIBRARIES} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING}
	                                                    ${RGBD_RTK_COMMON} ${RGBD_RTK_VISUALIZATION}
	                                                    ${RGBD_RTK_MOTION_ESTIMATION}
	                                                    ${RGBD_RTK_VISUAL_ODOMETRY} ${catkin_LIBRARIES})


install(TARGETS optical_flow_visual_odometry_test
				RUNTIME DESTINATION bin)

# Build SLAM related applications only if ARUCO was found
if(aruco_FOUND)
	add_executable(marker_finder_test marker_finder_test.cpp)
	find_library(RGBD_RTK_SLAM NAMES rgbd_rtk_slam)
	target_link_libraries(marker_finder_test ${PCL_LIBRARIES} ${RGBD_RTK_IO} ${RGBD_RTK_TRACKING}
										 ${RGBD_RTK_COMMON} ${RGBD_RTK_VISUALIZATION}
										 ${RGBD_RTK_MOTION_ESTIMATION}
										 ${RGBD_RTK_VISUAL_ODOMETRY}
										 ${RGBD_RTK_SLAM})
	install(TARGETS marker_finder_test
        RUNTIME DESTINATION bin)
endif()

# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/build

# Include any dependencies generated for this target.
include applications/CMakeFiles/optical_flow_visual_odometry_test.dir/depend.make

# Include the progress variables for this target.
include applications/CMakeFiles/optical_flow_visual_odometry_test.dir/progress.make

# Include the compile flags for this target's objects.
include applications/CMakeFiles/optical_flow_visual_odometry_test.dir/flags.make

applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o: applications/CMakeFiles/optical_flow_visual_odometry_test.dir/flags.make
applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o: ../applications/optical_flow_visual_odometry_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o"
	cd /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/build/applications && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o -c /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/applications/optical_flow_visual_odometry_test.cpp

applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.i"
	cd /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/build/applications && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/applications/optical_flow_visual_odometry_test.cpp > CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.i

applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.s"
	cd /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/build/applications && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/applications/optical_flow_visual_odometry_test.cpp -o CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.s

applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o.requires:

.PHONY : applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o.requires

applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o.provides: applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o.requires
	$(MAKE) -f applications/CMakeFiles/optical_flow_visual_odometry_test.dir/build.make applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o.provides.build
.PHONY : applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o.provides

applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o.provides.build: applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o


# Object files for target optical_flow_visual_odometry_test
optical_flow_visual_odometry_test_OBJECTS = \
"CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o"

# External object files for target optical_flow_visual_odometry_test
optical_flow_visual_odometry_test_EXTERNAL_OBJECTS =

applications/bin/optical_flow_visual_odometry_test: applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o
applications/bin/optical_flow_visual_odometry_test: applications/CMakeFiles/optical_flow_visual_odometry_test.dir/build.make
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_system.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_thread.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_regex.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libpthread.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_common.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_octree.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libOpenNI.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_io.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_kdtree.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_search.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_sample_consensus.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_filters.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_features.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_keypoints.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_registration.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_ml.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_recognition.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_surface.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_segmentation.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_visualization.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_people.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_outofcore.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_stereo.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_tracking.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_system.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_thread.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_regex.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libpthread.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libOpenNI.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
applications/bin/optical_flow_visual_odometry_test: io/lib/librgbd_rtk_io.so.1.0
applications/bin/optical_flow_visual_odometry_test: visualization/lib/librgbd_rtk_visualization.so.1.0
applications/bin/optical_flow_visual_odometry_test: visual_odometry/lib/librgbd_rtk_visual_odometry.so.1.0
applications/bin/optical_flow_visual_odometry_test: tracking/lib/librgbd_rtk_tracking.so.1.0
applications/bin/optical_flow_visual_odometry_test: motion_estimation/lib/librgbd_rtk_motion_estimation.so.1.0
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkGenericFiltering.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkGeovis.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkCharts.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkViews.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkInfovis.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkWidgets.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkVolumeRendering.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkHybrid.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkParallel.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkRendering.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkImaging.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkGraphics.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkIO.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkFiltering.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtkCommon.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libvtksys.so.5.10.1
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_system.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_thread.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_regex.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libpthread.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_common.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_octree.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libOpenNI.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_io.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_kdtree.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_search.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_sample_consensus.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_filters.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_features.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_keypoints.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_registration.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_ml.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_recognition.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_surface.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_segmentation.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_visualization.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_people.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_outofcore.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_stereo.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_tracking.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_system.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_thread.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libboost_regex.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libpthread.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_common.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_octree.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/libOpenNI.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_io.so
applications/bin/optical_flow_visual_odometry_test: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_kdtree.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_search.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_sample_consensus.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_filters.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_features.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_keypoints.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_registration.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_ml.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_recognition.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_surface.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_segmentation.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_visualization.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_people.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_outofcore.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_stereo.so
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libpcl_tracking.so
applications/bin/optical_flow_visual_odometry_test: common/lib/librgbd_rtk_common.so.1.0
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_shape.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_stitching.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_superres.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_ml.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_videostab.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_calib3d.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_features2d.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_flann.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_photo.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_video.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_highgui.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_videoio.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_imgcodecs.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_objdetect.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_imgproc.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: /usr/local/lib/libopencv_core.so.3.4.1
applications/bin/optical_flow_visual_odometry_test: applications/CMakeFiles/optical_flow_visual_odometry_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/optical_flow_visual_odometry_test"
	cd /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/build/applications && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/optical_flow_visual_odometry_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
applications/CMakeFiles/optical_flow_visual_odometry_test.dir/build: applications/bin/optical_flow_visual_odometry_test

.PHONY : applications/CMakeFiles/optical_flow_visual_odometry_test.dir/build

applications/CMakeFiles/optical_flow_visual_odometry_test.dir/requires: applications/CMakeFiles/optical_flow_visual_odometry_test.dir/optical_flow_visual_odometry_test.cpp.o.requires

.PHONY : applications/CMakeFiles/optical_flow_visual_odometry_test.dir/requires

applications/CMakeFiles/optical_flow_visual_odometry_test.dir/clean:
	cd /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/build/applications && $(CMAKE_COMMAND) -P CMakeFiles/optical_flow_visual_odometry_test.dir/cmake_clean.cmake
.PHONY : applications/CMakeFiles/optical_flow_visual_odometry_test.dir/clean

applications/CMakeFiles/optical_flow_visual_odometry_test.dir/depend:
	cd /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/applications /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/build /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/build/applications /home/ferreira/Documentos/rgbd_rtk/rgbd_rtk/build/applications/CMakeFiles/optical_flow_visual_odometry_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : applications/CMakeFiles/optical_flow_visual_odometry_test.dir/depend

# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/yons/projects/stereocamera

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yons/projects/stereocamera

# Include any dependencies generated for this target.
include CMakeFiles/stereo_camera.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stereo_camera.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereo_camera.dir/flags.make

CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o: CMakeFiles/stereo_camera.dir/flags.make
CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o: src/get_depth.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yons/projects/stereocamera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o -c /home/yons/projects/stereocamera/src/get_depth.cpp

CMakeFiles/stereo_camera.dir/src/get_depth.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_camera.dir/src/get_depth.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yons/projects/stereocamera/src/get_depth.cpp > CMakeFiles/stereo_camera.dir/src/get_depth.cpp.i

CMakeFiles/stereo_camera.dir/src/get_depth.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_camera.dir/src/get_depth.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yons/projects/stereocamera/src/get_depth.cpp -o CMakeFiles/stereo_camera.dir/src/get_depth.cpp.s

CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o.requires:

.PHONY : CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o.requires

CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o.provides: CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o.requires
	$(MAKE) -f CMakeFiles/stereo_camera.dir/build.make CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o.provides.build
.PHONY : CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o.provides

CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o.provides.build: CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o


CMakeFiles/stereo_camera.dir/src/main.cpp.o: CMakeFiles/stereo_camera.dir/flags.make
CMakeFiles/stereo_camera.dir/src/main.cpp.o: src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yons/projects/stereocamera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/stereo_camera.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_camera.dir/src/main.cpp.o -c /home/yons/projects/stereocamera/src/main.cpp

CMakeFiles/stereo_camera.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_camera.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yons/projects/stereocamera/src/main.cpp > CMakeFiles/stereo_camera.dir/src/main.cpp.i

CMakeFiles/stereo_camera.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_camera.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yons/projects/stereocamera/src/main.cpp -o CMakeFiles/stereo_camera.dir/src/main.cpp.s

CMakeFiles/stereo_camera.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/stereo_camera.dir/src/main.cpp.o.requires

CMakeFiles/stereo_camera.dir/src/main.cpp.o.provides: CMakeFiles/stereo_camera.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/stereo_camera.dir/build.make CMakeFiles/stereo_camera.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/stereo_camera.dir/src/main.cpp.o.provides

CMakeFiles/stereo_camera.dir/src/main.cpp.o.provides.build: CMakeFiles/stereo_camera.dir/src/main.cpp.o


CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o: CMakeFiles/stereo_camera.dir/flags.make
CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o: src/feature_extract.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yons/projects/stereocamera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o -c /home/yons/projects/stereocamera/src/feature_extract.cpp

CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yons/projects/stereocamera/src/feature_extract.cpp > CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.i

CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yons/projects/stereocamera/src/feature_extract.cpp -o CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.s

CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o.requires:

.PHONY : CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o.requires

CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o.provides: CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o.requires
	$(MAKE) -f CMakeFiles/stereo_camera.dir/build.make CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o.provides.build
.PHONY : CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o.provides

CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o.provides.build: CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o


CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o: CMakeFiles/stereo_camera.dir/flags.make
CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o: src/adjustimg.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yons/projects/stereocamera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o -c /home/yons/projects/stereocamera/src/adjustimg.cpp

CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yons/projects/stereocamera/src/adjustimg.cpp > CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.i

CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yons/projects/stereocamera/src/adjustimg.cpp -o CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.s

CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o.requires:

.PHONY : CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o.requires

CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o.provides: CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o.requires
	$(MAKE) -f CMakeFiles/stereo_camera.dir/build.make CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o.provides.build
.PHONY : CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o.provides

CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o.provides.build: CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o


CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o: CMakeFiles/stereo_camera.dir/flags.make
CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o: src/image_stitch.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yons/projects/stereocamera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o -c /home/yons/projects/stereocamera/src/image_stitch.cpp

CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yons/projects/stereocamera/src/image_stitch.cpp > CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.i

CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yons/projects/stereocamera/src/image_stitch.cpp -o CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.s

CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o.requires:

.PHONY : CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o.requires

CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o.provides: CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o.requires
	$(MAKE) -f CMakeFiles/stereo_camera.dir/build.make CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o.provides.build
.PHONY : CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o.provides

CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o.provides.build: CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o


CMakeFiles/stereo_camera.dir/src/calibration.cpp.o: CMakeFiles/stereo_camera.dir/flags.make
CMakeFiles/stereo_camera.dir/src/calibration.cpp.o: src/calibration.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yons/projects/stereocamera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/stereo_camera.dir/src/calibration.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_camera.dir/src/calibration.cpp.o -c /home/yons/projects/stereocamera/src/calibration.cpp

CMakeFiles/stereo_camera.dir/src/calibration.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_camera.dir/src/calibration.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yons/projects/stereocamera/src/calibration.cpp > CMakeFiles/stereo_camera.dir/src/calibration.cpp.i

CMakeFiles/stereo_camera.dir/src/calibration.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_camera.dir/src/calibration.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yons/projects/stereocamera/src/calibration.cpp -o CMakeFiles/stereo_camera.dir/src/calibration.cpp.s

CMakeFiles/stereo_camera.dir/src/calibration.cpp.o.requires:

.PHONY : CMakeFiles/stereo_camera.dir/src/calibration.cpp.o.requires

CMakeFiles/stereo_camera.dir/src/calibration.cpp.o.provides: CMakeFiles/stereo_camera.dir/src/calibration.cpp.o.requires
	$(MAKE) -f CMakeFiles/stereo_camera.dir/build.make CMakeFiles/stereo_camera.dir/src/calibration.cpp.o.provides.build
.PHONY : CMakeFiles/stereo_camera.dir/src/calibration.cpp.o.provides

CMakeFiles/stereo_camera.dir/src/calibration.cpp.o.provides.build: CMakeFiles/stereo_camera.dir/src/calibration.cpp.o


# Object files for target stereo_camera
stereo_camera_OBJECTS = \
"CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o" \
"CMakeFiles/stereo_camera.dir/src/main.cpp.o" \
"CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o" \
"CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o" \
"CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o" \
"CMakeFiles/stereo_camera.dir/src/calibration.cpp.o"

# External object files for target stereo_camera
stereo_camera_EXTERNAL_OBJECTS =

bin/stereo_camera: CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o
bin/stereo_camera: CMakeFiles/stereo_camera.dir/src/main.cpp.o
bin/stereo_camera: CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o
bin/stereo_camera: CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o
bin/stereo_camera: CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o
bin/stereo_camera: CMakeFiles/stereo_camera.dir/src/calibration.cpp.o
bin/stereo_camera: CMakeFiles/stereo_camera.dir/build.make
bin/stereo_camera: /usr/local/lib/libopencv_xphoto.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_xobjdetect.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_tracking.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_surface_matching.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_structured_light.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_stereo.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_saliency.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_rgbd.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_reg.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_plot.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_optflow.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_line_descriptor.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_hdf.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_fuzzy.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_dpm.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_dnn.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_datasets.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_ccalib.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_bioinspired.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_bgsegm.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_aruco.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_viz.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_videostab.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_superres.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_stitching.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_photo.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_text.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_face.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_ximgproc.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_xfeatures2d.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_shape.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_video.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_objdetect.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_calib3d.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_features2d.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_ml.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_highgui.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_videoio.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_imgproc.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_flann.so.3.1.0
bin/stereo_camera: /usr/local/lib/libopencv_core.so.3.1.0
bin/stereo_camera: CMakeFiles/stereo_camera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yons/projects/stereocamera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable bin/stereo_camera"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo_camera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereo_camera.dir/build: bin/stereo_camera

.PHONY : CMakeFiles/stereo_camera.dir/build

CMakeFiles/stereo_camera.dir/requires: CMakeFiles/stereo_camera.dir/src/get_depth.cpp.o.requires
CMakeFiles/stereo_camera.dir/requires: CMakeFiles/stereo_camera.dir/src/main.cpp.o.requires
CMakeFiles/stereo_camera.dir/requires: CMakeFiles/stereo_camera.dir/src/feature_extract.cpp.o.requires
CMakeFiles/stereo_camera.dir/requires: CMakeFiles/stereo_camera.dir/src/adjustimg.cpp.o.requires
CMakeFiles/stereo_camera.dir/requires: CMakeFiles/stereo_camera.dir/src/image_stitch.cpp.o.requires
CMakeFiles/stereo_camera.dir/requires: CMakeFiles/stereo_camera.dir/src/calibration.cpp.o.requires

.PHONY : CMakeFiles/stereo_camera.dir/requires

CMakeFiles/stereo_camera.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereo_camera.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereo_camera.dir/clean

CMakeFiles/stereo_camera.dir/depend:
	cd /home/yons/projects/stereocamera && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yons/projects/stereocamera /home/yons/projects/stereocamera /home/yons/projects/stereocamera /home/yons/projects/stereocamera /home/yons/projects/stereocamera/CMakeFiles/stereo_camera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stereo_camera.dir/depend


# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.11

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
CMAKE_SOURCE_DIR = /home/alex/Work/OpenSource/LitterBug-Algorithm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alex/Work/OpenSource/LitterBug-Algorithm/build-cmake

# Include any dependencies generated for this target.
include CMakeFiles/litter_bug_nogui.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/litter_bug_nogui.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/litter_bug_nogui.dir/flags.make

CMakeFiles/litter_bug_nogui.dir/Litter_detect.cpp.o: CMakeFiles/litter_bug_nogui.dir/flags.make
CMakeFiles/litter_bug_nogui.dir/Litter_detect.cpp.o: ../Litter_detect.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alex/Work/OpenSource/LitterBug-Algorithm/build-cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/litter_bug_nogui.dir/Litter_detect.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/litter_bug_nogui.dir/Litter_detect.cpp.o -c /home/alex/Work/OpenSource/LitterBug-Algorithm/Litter_detect.cpp

CMakeFiles/litter_bug_nogui.dir/Litter_detect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/litter_bug_nogui.dir/Litter_detect.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alex/Work/OpenSource/LitterBug-Algorithm/Litter_detect.cpp > CMakeFiles/litter_bug_nogui.dir/Litter_detect.cpp.i

CMakeFiles/litter_bug_nogui.dir/Litter_detect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/litter_bug_nogui.dir/Litter_detect.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alex/Work/OpenSource/LitterBug-Algorithm/Litter_detect.cpp -o CMakeFiles/litter_bug_nogui.dir/Litter_detect.cpp.s

CMakeFiles/litter_bug_nogui.dir/edge_grouping.cpp.o: CMakeFiles/litter_bug_nogui.dir/flags.make
CMakeFiles/litter_bug_nogui.dir/edge_grouping.cpp.o: ../edge_grouping.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alex/Work/OpenSource/LitterBug-Algorithm/build-cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/litter_bug_nogui.dir/edge_grouping.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/litter_bug_nogui.dir/edge_grouping.cpp.o -c /home/alex/Work/OpenSource/LitterBug-Algorithm/edge_grouping.cpp

CMakeFiles/litter_bug_nogui.dir/edge_grouping.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/litter_bug_nogui.dir/edge_grouping.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alex/Work/OpenSource/LitterBug-Algorithm/edge_grouping.cpp > CMakeFiles/litter_bug_nogui.dir/edge_grouping.cpp.i

CMakeFiles/litter_bug_nogui.dir/edge_grouping.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/litter_bug_nogui.dir/edge_grouping.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alex/Work/OpenSource/LitterBug-Algorithm/edge_grouping.cpp -o CMakeFiles/litter_bug_nogui.dir/edge_grouping.cpp.s

CMakeFiles/litter_bug_nogui.dir/scoring.cpp.o: CMakeFiles/litter_bug_nogui.dir/flags.make
CMakeFiles/litter_bug_nogui.dir/scoring.cpp.o: ../scoring.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alex/Work/OpenSource/LitterBug-Algorithm/build-cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/litter_bug_nogui.dir/scoring.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/litter_bug_nogui.dir/scoring.cpp.o -c /home/alex/Work/OpenSource/LitterBug-Algorithm/scoring.cpp

CMakeFiles/litter_bug_nogui.dir/scoring.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/litter_bug_nogui.dir/scoring.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alex/Work/OpenSource/LitterBug-Algorithm/scoring.cpp > CMakeFiles/litter_bug_nogui.dir/scoring.cpp.i

CMakeFiles/litter_bug_nogui.dir/scoring.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/litter_bug_nogui.dir/scoring.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alex/Work/OpenSource/LitterBug-Algorithm/scoring.cpp -o CMakeFiles/litter_bug_nogui.dir/scoring.cpp.s

# Object files for target litter_bug_nogui
litter_bug_nogui_OBJECTS = \
"CMakeFiles/litter_bug_nogui.dir/Litter_detect.cpp.o" \
"CMakeFiles/litter_bug_nogui.dir/edge_grouping.cpp.o" \
"CMakeFiles/litter_bug_nogui.dir/scoring.cpp.o"

# External object files for target litter_bug_nogui
litter_bug_nogui_EXTERNAL_OBJECTS =

litter_bug_nogui: CMakeFiles/litter_bug_nogui.dir/Litter_detect.cpp.o
litter_bug_nogui: CMakeFiles/litter_bug_nogui.dir/edge_grouping.cpp.o
litter_bug_nogui: CMakeFiles/litter_bug_nogui.dir/scoring.cpp.o
litter_bug_nogui: CMakeFiles/litter_bug_nogui.dir/build.make
litter_bug_nogui: /usr/lib/libopencv_stitching.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_superres.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_videostab.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_aruco.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_bgsegm.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_bioinspired.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_ccalib.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_dnn_objdetect.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_dpm.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_face.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_freetype.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_fuzzy.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_hdf.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_hfs.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_img_hash.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_line_descriptor.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_optflow.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_reg.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_rgbd.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_saliency.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_stereo.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_structured_light.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_surface_matching.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_tracking.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_xfeatures2d.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_ximgproc.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_xobjdetect.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_xphoto.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_shape.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_photo.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_datasets.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_plot.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_text.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_dnn.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_ml.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_video.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_calib3d.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_features2d.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_highgui.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_videoio.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_phase_unwrapping.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_flann.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_imgcodecs.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_objdetect.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_imgproc.so.3.4.1
litter_bug_nogui: /usr/lib/libopencv_core.so.3.4.1
litter_bug_nogui: CMakeFiles/litter_bug_nogui.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alex/Work/OpenSource/LitterBug-Algorithm/build-cmake/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable litter_bug_nogui"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/litter_bug_nogui.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/litter_bug_nogui.dir/build: litter_bug_nogui

.PHONY : CMakeFiles/litter_bug_nogui.dir/build

CMakeFiles/litter_bug_nogui.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/litter_bug_nogui.dir/cmake_clean.cmake
.PHONY : CMakeFiles/litter_bug_nogui.dir/clean

CMakeFiles/litter_bug_nogui.dir/depend:
	cd /home/alex/Work/OpenSource/LitterBug-Algorithm/build-cmake && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alex/Work/OpenSource/LitterBug-Algorithm /home/alex/Work/OpenSource/LitterBug-Algorithm /home/alex/Work/OpenSource/LitterBug-Algorithm/build-cmake /home/alex/Work/OpenSource/LitterBug-Algorithm/build-cmake /home/alex/Work/OpenSource/LitterBug-Algorithm/build-cmake/CMakeFiles/litter_bug_nogui.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/litter_bug_nogui.dir/depend


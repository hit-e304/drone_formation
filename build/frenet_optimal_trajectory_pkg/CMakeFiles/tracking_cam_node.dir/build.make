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
CMAKE_COMMAND = /home/zhan/cmake-3.10.2/bin/cmake

# The command to remove a file.
RM = /home/zhan/cmake-3.10.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zhan/drone_formation/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zhan/drone_formation/build

# Include any dependencies generated for this target.
include frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/depend.make

# Include the progress variables for this target.
include frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/progress.make

# Include the compile flags for this target's objects.
include frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/flags.make

frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o: frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/flags.make
frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o: /home/zhan/drone_formation/src/frenet_optimal_trajectory_pkg/src/tracking_cam_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhan/drone_formation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o"
	cd /home/zhan/drone_formation/build/frenet_optimal_trajectory_pkg && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o -c /home/zhan/drone_formation/src/frenet_optimal_trajectory_pkg/src/tracking_cam_node.cpp

frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.i"
	cd /home/zhan/drone_formation/build/frenet_optimal_trajectory_pkg && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhan/drone_formation/src/frenet_optimal_trajectory_pkg/src/tracking_cam_node.cpp > CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.i

frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.s"
	cd /home/zhan/drone_formation/build/frenet_optimal_trajectory_pkg && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhan/drone_formation/src/frenet_optimal_trajectory_pkg/src/tracking_cam_node.cpp -o CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.s

frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o.requires:

.PHONY : frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o.requires

frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o.provides: frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o.requires
	$(MAKE) -f frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/build.make frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o.provides.build
.PHONY : frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o.provides

frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o.provides.build: frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o


# Object files for target tracking_cam_node
tracking_cam_node_OBJECTS = \
"CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o"

# External object files for target tracking_cam_node
tracking_cam_node_EXTERNAL_OBJECTS =

/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/build.make
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /opt/ros/kinetic/lib/libroscpp.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /opt/ros/kinetic/lib/librosconsole.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /opt/ros/kinetic/lib/librostime.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /opt/ros/kinetic/lib/libcpp_common.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node: frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhan/drone_formation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node"
	cd /home/zhan/drone_formation/build/frenet_optimal_trajectory_pkg && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tracking_cam_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/build: /home/zhan/drone_formation/devel/lib/frenet_optimal_trajectory_pkg/tracking_cam_node

.PHONY : frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/build

frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/requires: frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/src/tracking_cam_node.cpp.o.requires

.PHONY : frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/requires

frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/clean:
	cd /home/zhan/drone_formation/build/frenet_optimal_trajectory_pkg && $(CMAKE_COMMAND) -P CMakeFiles/tracking_cam_node.dir/cmake_clean.cmake
.PHONY : frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/clean

frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/depend:
	cd /home/zhan/drone_formation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhan/drone_formation/src /home/zhan/drone_formation/src/frenet_optimal_trajectory_pkg /home/zhan/drone_formation/build /home/zhan/drone_formation/build/frenet_optimal_trajectory_pkg /home/zhan/drone_formation/build/frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : frenet_optimal_trajectory_pkg/CMakeFiles/tracking_cam_node.dir/depend


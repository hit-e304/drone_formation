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
CMAKE_SOURCE_DIR = /home/lihai/drone_test_ws/drone_formation/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lihai/drone_test_ws/drone_formation/build

# Include any dependencies generated for this target.
include formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/depend.make

# Include the progress variables for this target.
include formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/progress.make

# Include the compile flags for this target's objects.
include formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/flags.make

formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o: formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/flags.make
formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o: /home/lihai/drone_test_ws/drone_formation/src/formation_algorithm_leader_follower/src/UAV2_follower_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lihai/drone_test_ws/drone_formation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o"
	cd /home/lihai/drone_test_ws/drone_formation/build/formation_algorithm_leader_follower && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o -c /home/lihai/drone_test_ws/drone_formation/src/formation_algorithm_leader_follower/src/UAV2_follower_node.cpp

formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.i"
	cd /home/lihai/drone_test_ws/drone_formation/build/formation_algorithm_leader_follower && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lihai/drone_test_ws/drone_formation/src/formation_algorithm_leader_follower/src/UAV2_follower_node.cpp > CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.i

formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.s"
	cd /home/lihai/drone_test_ws/drone_formation/build/formation_algorithm_leader_follower && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lihai/drone_test_ws/drone_formation/src/formation_algorithm_leader_follower/src/UAV2_follower_node.cpp -o CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.s

formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o.requires:

.PHONY : formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o.requires

formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o.provides: formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o.requires
	$(MAKE) -f formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/build.make formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o.provides.build
.PHONY : formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o.provides

formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o.provides.build: formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o


# Object files for target UAV2_follower_node
UAV2_follower_node_OBJECTS = \
"CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o"

# External object files for target UAV2_follower_node
UAV2_follower_node_EXTERNAL_OBJECTS =

/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/build.make
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /opt/ros/kinetic/lib/libroscpp.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /opt/ros/kinetic/lib/librosconsole.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /opt/ros/kinetic/lib/librostime.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /opt/ros/kinetic/lib/libcpp_common.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node: formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lihai/drone_test_ws/drone_formation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node"
	cd /home/lihai/drone_test_ws/drone_formation/build/formation_algorithm_leader_follower && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/UAV2_follower_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/build: /home/lihai/drone_test_ws/drone_formation/devel/lib/formation_algorithm_leader_follower/UAV2_follower_node

.PHONY : formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/build

formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/requires: formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/src/UAV2_follower_node.cpp.o.requires

.PHONY : formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/requires

formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/clean:
	cd /home/lihai/drone_test_ws/drone_formation/build/formation_algorithm_leader_follower && $(CMAKE_COMMAND) -P CMakeFiles/UAV2_follower_node.dir/cmake_clean.cmake
.PHONY : formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/clean

formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/depend:
	cd /home/lihai/drone_test_ws/drone_formation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lihai/drone_test_ws/drone_formation/src /home/lihai/drone_test_ws/drone_formation/src/formation_algorithm_leader_follower /home/lihai/drone_test_ws/drone_formation/build /home/lihai/drone_test_ws/drone_formation/build/formation_algorithm_leader_follower /home/lihai/drone_test_ws/drone_formation/build/formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : formation_algorithm_leader_follower/CMakeFiles/UAV2_follower_node.dir/depend


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
include formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/depend.make

# Include the progress variables for this target.
include formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/progress.make

# Include the compile flags for this target's objects.
include formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/flags.make

formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o: formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/flags.make
formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o: /home/lihai/drone_test_ws/drone_formation/src/formation_algorithm_leader_follower/src/CubicSplinePlanner.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lihai/drone_test_ws/drone_formation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o"
	cd /home/lihai/drone_test_ws/drone_formation/build/formation_algorithm_leader_follower && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o -c /home/lihai/drone_test_ws/drone_formation/src/formation_algorithm_leader_follower/src/CubicSplinePlanner.cpp

formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.i"
	cd /home/lihai/drone_test_ws/drone_formation/build/formation_algorithm_leader_follower && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lihai/drone_test_ws/drone_formation/src/formation_algorithm_leader_follower/src/CubicSplinePlanner.cpp > CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.i

formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.s"
	cd /home/lihai/drone_test_ws/drone_formation/build/formation_algorithm_leader_follower && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lihai/drone_test_ws/drone_formation/src/formation_algorithm_leader_follower/src/CubicSplinePlanner.cpp -o CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.s

formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o.requires:

.PHONY : formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o.requires

formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o.provides: formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o.requires
	$(MAKE) -f formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/build.make formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o.provides.build
.PHONY : formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o.provides

formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o.provides.build: formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o


# Object files for target CubicSplinePlanner_LF
CubicSplinePlanner_LF_OBJECTS = \
"CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o"

# External object files for target CubicSplinePlanner_LF
CubicSplinePlanner_LF_EXTERNAL_OBJECTS =

/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/build.make
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /opt/ros/kinetic/lib/libroscpp.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /opt/ros/kinetic/lib/librosconsole.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /opt/ros/kinetic/lib/librostime.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /opt/ros/kinetic/lib/libcpp_common.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so: formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lihai/drone_test_ws/drone_formation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so"
	cd /home/lihai/drone_test_ws/drone_formation/build/formation_algorithm_leader_follower && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/CubicSplinePlanner_LF.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/build: /home/lihai/drone_test_ws/drone_formation/devel/lib/libCubicSplinePlanner_LF.so

.PHONY : formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/build

formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/requires: formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/src/CubicSplinePlanner.cpp.o.requires

.PHONY : formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/requires

formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/clean:
	cd /home/lihai/drone_test_ws/drone_formation/build/formation_algorithm_leader_follower && $(CMAKE_COMMAND) -P CMakeFiles/CubicSplinePlanner_LF.dir/cmake_clean.cmake
.PHONY : formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/clean

formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/depend:
	cd /home/lihai/drone_test_ws/drone_formation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lihai/drone_test_ws/drone_formation/src /home/lihai/drone_test_ws/drone_formation/src/formation_algorithm_leader_follower /home/lihai/drone_test_ws/drone_formation/build /home/lihai/drone_test_ws/drone_formation/build/formation_algorithm_leader_follower /home/lihai/drone_test_ws/drone_formation/build/formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : formation_algorithm_leader_follower/CMakeFiles/CubicSplinePlanner_LF.dir/depend

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

# Utility rule file for opencvtest_gencpp.

# Include the progress variables for this target.
include opencvtest/CMakeFiles/opencvtest_gencpp.dir/progress.make

opencvtest_gencpp: opencvtest/CMakeFiles/opencvtest_gencpp.dir/build.make

.PHONY : opencvtest_gencpp

# Rule to build all files generated by this target.
opencvtest/CMakeFiles/opencvtest_gencpp.dir/build: opencvtest_gencpp

.PHONY : opencvtest/CMakeFiles/opencvtest_gencpp.dir/build

opencvtest/CMakeFiles/opencvtest_gencpp.dir/clean:
	cd /home/lihai/drone_test_ws/drone_formation/build/opencvtest && $(CMAKE_COMMAND) -P CMakeFiles/opencvtest_gencpp.dir/cmake_clean.cmake
.PHONY : opencvtest/CMakeFiles/opencvtest_gencpp.dir/clean

opencvtest/CMakeFiles/opencvtest_gencpp.dir/depend:
	cd /home/lihai/drone_test_ws/drone_formation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lihai/drone_test_ws/drone_formation/src /home/lihai/drone_test_ws/drone_formation/src/opencvtest /home/lihai/drone_test_ws/drone_formation/build /home/lihai/drone_test_ws/drone_formation/build/opencvtest /home/lihai/drone_test_ws/drone_formation/build/opencvtest/CMakeFiles/opencvtest_gencpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : opencvtest/CMakeFiles/opencvtest_gencpp.dir/depend


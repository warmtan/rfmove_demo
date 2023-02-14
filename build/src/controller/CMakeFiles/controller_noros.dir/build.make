# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/robotflow/rfmove

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robotflow/rfmove/build

# Include any dependencies generated for this target.
include src/controller/CMakeFiles/controller_noros.dir/depend.make

# Include the progress variables for this target.
include src/controller/CMakeFiles/controller_noros.dir/progress.make

# Include the compile flags for this target's objects.
include src/controller/CMakeFiles/controller_noros.dir/flags.make

src/controller/CMakeFiles/controller_noros.dir/src/trajectory.cpp.o: src/controller/CMakeFiles/controller_noros.dir/flags.make
src/controller/CMakeFiles/controller_noros.dir/src/trajectory.cpp.o: ../src/controller/src/trajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robotflow/rfmove/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/controller/CMakeFiles/controller_noros.dir/src/trajectory.cpp.o"
	cd /home/robotflow/rfmove/build/src/controller && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/controller_noros.dir/src/trajectory.cpp.o -c /home/robotflow/rfmove/src/controller/src/trajectory.cpp

src/controller/CMakeFiles/controller_noros.dir/src/trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controller_noros.dir/src/trajectory.cpp.i"
	cd /home/robotflow/rfmove/build/src/controller && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robotflow/rfmove/src/controller/src/trajectory.cpp > CMakeFiles/controller_noros.dir/src/trajectory.cpp.i

src/controller/CMakeFiles/controller_noros.dir/src/trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controller_noros.dir/src/trajectory.cpp.s"
	cd /home/robotflow/rfmove/build/src/controller && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robotflow/rfmove/src/controller/src/trajectory.cpp -o CMakeFiles/controller_noros.dir/src/trajectory.cpp.s

src/controller/CMakeFiles/controller_noros.dir/src/trajectory_controller.cpp.o: src/controller/CMakeFiles/controller_noros.dir/flags.make
src/controller/CMakeFiles/controller_noros.dir/src/trajectory_controller.cpp.o: ../src/controller/src/trajectory_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robotflow/rfmove/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/controller/CMakeFiles/controller_noros.dir/src/trajectory_controller.cpp.o"
	cd /home/robotflow/rfmove/build/src/controller && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/controller_noros.dir/src/trajectory_controller.cpp.o -c /home/robotflow/rfmove/src/controller/src/trajectory_controller.cpp

src/controller/CMakeFiles/controller_noros.dir/src/trajectory_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controller_noros.dir/src/trajectory_controller.cpp.i"
	cd /home/robotflow/rfmove/build/src/controller && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robotflow/rfmove/src/controller/src/trajectory_controller.cpp > CMakeFiles/controller_noros.dir/src/trajectory_controller.cpp.i

src/controller/CMakeFiles/controller_noros.dir/src/trajectory_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controller_noros.dir/src/trajectory_controller.cpp.s"
	cd /home/robotflow/rfmove/build/src/controller && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robotflow/rfmove/src/controller/src/trajectory_controller.cpp -o CMakeFiles/controller_noros.dir/src/trajectory_controller.cpp.s

src/controller/CMakeFiles/controller_noros.dir/src/hardware.cpp.o: src/controller/CMakeFiles/controller_noros.dir/flags.make
src/controller/CMakeFiles/controller_noros.dir/src/hardware.cpp.o: ../src/controller/src/hardware.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robotflow/rfmove/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/controller/CMakeFiles/controller_noros.dir/src/hardware.cpp.o"
	cd /home/robotflow/rfmove/build/src/controller && /usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/controller_noros.dir/src/hardware.cpp.o -c /home/robotflow/rfmove/src/controller/src/hardware.cpp

src/controller/CMakeFiles/controller_noros.dir/src/hardware.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/controller_noros.dir/src/hardware.cpp.i"
	cd /home/robotflow/rfmove/build/src/controller && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robotflow/rfmove/src/controller/src/hardware.cpp > CMakeFiles/controller_noros.dir/src/hardware.cpp.i

src/controller/CMakeFiles/controller_noros.dir/src/hardware.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/controller_noros.dir/src/hardware.cpp.s"
	cd /home/robotflow/rfmove/build/src/controller && /usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robotflow/rfmove/src/controller/src/hardware.cpp -o CMakeFiles/controller_noros.dir/src/hardware.cpp.s

# Object files for target controller_noros
controller_noros_OBJECTS = \
"CMakeFiles/controller_noros.dir/src/trajectory.cpp.o" \
"CMakeFiles/controller_noros.dir/src/trajectory_controller.cpp.o" \
"CMakeFiles/controller_noros.dir/src/hardware.cpp.o"

# External object files for target controller_noros
controller_noros_EXTERNAL_OBJECTS =

src/controller/libcontroller_noros.so: src/controller/CMakeFiles/controller_noros.dir/src/trajectory.cpp.o
src/controller/libcontroller_noros.so: src/controller/CMakeFiles/controller_noros.dir/src/trajectory_controller.cpp.o
src/controller/libcontroller_noros.so: src/controller/CMakeFiles/controller_noros.dir/src/hardware.cpp.o
src/controller/libcontroller_noros.so: src/controller/CMakeFiles/controller_noros.dir/build.make
src/controller/libcontroller_noros.so: ../extern/lib/libmoveit_robot_trajectory.so
src/controller/libcontroller_noros.so: ../extern/lib/libmoveit_trajectory_processing.so
src/controller/libcontroller_noros.so: src/controller/CMakeFiles/controller_noros.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robotflow/rfmove/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library libcontroller_noros.so"
	cd /home/robotflow/rfmove/build/src/controller && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/controller_noros.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/controller/CMakeFiles/controller_noros.dir/build: src/controller/libcontroller_noros.so

.PHONY : src/controller/CMakeFiles/controller_noros.dir/build

src/controller/CMakeFiles/controller_noros.dir/clean:
	cd /home/robotflow/rfmove/build/src/controller && $(CMAKE_COMMAND) -P CMakeFiles/controller_noros.dir/cmake_clean.cmake
.PHONY : src/controller/CMakeFiles/controller_noros.dir/clean

src/controller/CMakeFiles/controller_noros.dir/depend:
	cd /home/robotflow/rfmove/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robotflow/rfmove /home/robotflow/rfmove/src/controller /home/robotflow/rfmove/build /home/robotflow/rfmove/build/src/controller /home/robotflow/rfmove/build/src/controller/CMakeFiles/controller_noros.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/controller/CMakeFiles/controller_noros.dir/depend


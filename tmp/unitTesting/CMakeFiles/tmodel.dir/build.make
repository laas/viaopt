# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Produce verbose output by default.
VERBOSE = 1

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pgeoffro/src/viaopt

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pgeoffro/src/viaopt/tmp

# Include any dependencies generated for this target.
include unitTesting/CMakeFiles/tmodel.dir/depend.make

# Include the progress variables for this target.
include unitTesting/CMakeFiles/tmodel.dir/progress.make

# Include the compile flags for this target's objects.
include unitTesting/CMakeFiles/tmodel.dir/flags.make

unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.o: unitTesting/CMakeFiles/tmodel.dir/flags.make
unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.o: ../unitTesting/tmodel.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pgeoffro/src/viaopt/tmp/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.o"
	cd /home/pgeoffro/src/viaopt/tmp/unitTesting && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/tmodel.dir/tmodel.cpp.o -c /home/pgeoffro/src/viaopt/unitTesting/tmodel.cpp

unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tmodel.dir/tmodel.cpp.i"
	cd /home/pgeoffro/src/viaopt/tmp/unitTesting && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/pgeoffro/src/viaopt/unitTesting/tmodel.cpp > CMakeFiles/tmodel.dir/tmodel.cpp.i

unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tmodel.dir/tmodel.cpp.s"
	cd /home/pgeoffro/src/viaopt/tmp/unitTesting && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/pgeoffro/src/viaopt/unitTesting/tmodel.cpp -o CMakeFiles/tmodel.dir/tmodel.cpp.s

unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.o.requires:
.PHONY : unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.o.requires

unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.o.provides: unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.o.requires
	$(MAKE) -f unitTesting/CMakeFiles/tmodel.dir/build.make unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.o.provides.build
.PHONY : unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.o.provides

unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.o.provides.build: unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.o
.PHONY : unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.o.provides.build

# Object files for target tmodel
tmodel_OBJECTS = \
"CMakeFiles/tmodel.dir/tmodel.cpp.o"

# External object files for target tmodel
tmodel_EXTERNAL_OBJECTS =

unitTesting/tmodel: unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.o
unitTesting/tmodel: src/libviaopt.so.0.0-1-g35f5-dirty
unitTesting/tmodel: unitTesting/CMakeFiles/tmodel.dir/build.make
unitTesting/tmodel: unitTesting/CMakeFiles/tmodel.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable tmodel"
	cd /home/pgeoffro/src/viaopt/tmp/unitTesting && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tmodel.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
unitTesting/CMakeFiles/tmodel.dir/build: unitTesting/tmodel
.PHONY : unitTesting/CMakeFiles/tmodel.dir/build

unitTesting/CMakeFiles/tmodel.dir/requires: unitTesting/CMakeFiles/tmodel.dir/tmodel.cpp.o.requires
.PHONY : unitTesting/CMakeFiles/tmodel.dir/requires

unitTesting/CMakeFiles/tmodel.dir/clean:
	cd /home/pgeoffro/src/viaopt/tmp/unitTesting && $(CMAKE_COMMAND) -P CMakeFiles/tmodel.dir/cmake_clean.cmake
.PHONY : unitTesting/CMakeFiles/tmodel.dir/clean

unitTesting/CMakeFiles/tmodel.dir/depend:
	cd /home/pgeoffro/src/viaopt/tmp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pgeoffro/src/viaopt /home/pgeoffro/src/viaopt/unitTesting /home/pgeoffro/src/viaopt/tmp /home/pgeoffro/src/viaopt/tmp/unitTesting /home/pgeoffro/src/viaopt/tmp/unitTesting/CMakeFiles/tmodel.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : unitTesting/CMakeFiles/tmodel.dir/depend


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

# Utility rule file for distcheck.

CMakeFiles/distcheck:
	$(CMAKE_COMMAND) -E cmake_progress_report /home/pgeoffro/src/viaopt/tmp/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Checking generated tarball..."
	cd /home/pgeoffro/src/viaopt/tmp/viaopt-0.0-1-g35f5-dirty && find . -type d -print0 | xargs -0 chmod a-w && chmod u+w . && rm -rf _build _inst && mkdir -p _build && mkdir -p _inst && chmod u+rwx _build _inst && chmod a-w . && cp /home/pgeoffro/src/viaopt/tmp/CMakeCache.txt _build/ && /bin/sed -i -e 's|CMAKE_CACHEFILE_DIR:INTERNAL=.\+||g' -e 's|CMAKE_HOME_DIRECTORY:INTERNAL=.\+||g' _build/CMakeCache.txt && cd _build && cmake -DCMAKE_INSTALL_PREFIX=/home/pgeoffro/src/viaopt/tmp/viaopt-0.0-1-g35f5-dirty/_inst .. || cmake .. || ( echo ERROR:\ the\ cmake\ configuration\ failed. && false ) && make || ( echo ERROR:\ the\ compilation\ failed. && false ) && make test || ( echo ERROR:\ the\ test\ suite\ failed. && false ) && make install || ( echo ERROR:\ the\ install\ target\ failed. && false ) && make uninstall || ( echo ERROR:\ the\ uninstall\ target\ failed. && false ) && test x`find /home/pgeoffro/src/viaopt/tmp/viaopt-0.0-1-g35f5-dirty/_inst -type f | wc -l` = x0 || ( echo ERROR:\ the\ uninstall\ target\ does\ not\ work. && false ) && make clean || ( echo ERROR:\ the\ clean\ target\ failed. && false ) && cd /home/pgeoffro/src/viaopt/tmp/viaopt-0.0-1-g35f5-dirty && chmod u+w . _build _inst && rm -rf _build _inst && find . -type d -print0 | xargs -0 chmod u+w && echo ============================================================== && echo viaopt-0.0-1-g35f5-dirty is\ ready\ for\ distribution. && echo ==============================================================

distcheck: CMakeFiles/distcheck
distcheck: CMakeFiles/distcheck.dir/build.make
.PHONY : distcheck

# Rule to build all files generated by this target.
CMakeFiles/distcheck.dir/build: distcheck
.PHONY : CMakeFiles/distcheck.dir/build

CMakeFiles/distcheck.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/distcheck.dir/cmake_clean.cmake
.PHONY : CMakeFiles/distcheck.dir/clean

CMakeFiles/distcheck.dir/depend:
	cd /home/pgeoffro/src/viaopt/tmp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pgeoffro/src/viaopt /home/pgeoffro/src/viaopt /home/pgeoffro/src/viaopt/tmp /home/pgeoffro/src/viaopt/tmp /home/pgeoffro/src/viaopt/tmp/CMakeFiles/distcheck.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/distcheck.dir/depend


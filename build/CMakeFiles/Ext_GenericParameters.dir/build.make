# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/paula/Desktop/PBD/PositionBasedDynamics

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/paula/Desktop/PBD/PositionBasedDynamics/build

# Utility rule file for Ext_GenericParameters.

# Include any custom commands dependencies for this target.
include CMakeFiles/Ext_GenericParameters.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Ext_GenericParameters.dir/progress.make

CMakeFiles/Ext_GenericParameters: CMakeFiles/Ext_GenericParameters-complete

CMakeFiles/Ext_GenericParameters-complete: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-install
CMakeFiles/Ext_GenericParameters-complete: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-mkdir
CMakeFiles/Ext_GenericParameters-complete: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-download
CMakeFiles/Ext_GenericParameters-complete: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-update
CMakeFiles/Ext_GenericParameters-complete: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-patch
CMakeFiles/Ext_GenericParameters-complete: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-configure
CMakeFiles/Ext_GenericParameters-complete: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-build
CMakeFiles/Ext_GenericParameters-complete: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/paula/Desktop/PBD/PositionBasedDynamics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'Ext_GenericParameters'"
	/usr/bin/cmake -E make_directory /home/paula/Desktop/PBD/PositionBasedDynamics/build/CMakeFiles
	/usr/bin/cmake -E touch /home/paula/Desktop/PBD/PositionBasedDynamics/build/CMakeFiles/Ext_GenericParameters-complete
	/usr/bin/cmake -E touch /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-done

GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-build: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/paula/Desktop/PBD/PositionBasedDynamics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing build step for 'Ext_GenericParameters'"
	cd /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-build && $(MAKE)
	cd /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-build && /usr/bin/cmake -E touch /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-build

GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-configure: GenericParameters/tmp/Ext_GenericParameters-cfgcmd.txt
GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-configure: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/paula/Desktop/PBD/PositionBasedDynamics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Performing configure step for 'Ext_GenericParameters'"
	cd /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-build && /usr/bin/cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX:PATH=/home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters -DGENERICPARAMETERS_NO_TESTS:BOOL=1 "-GUnix Makefiles" /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters
	cd /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-build && /usr/bin/cmake -E touch /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-configure

GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-download: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitinfo.txt
GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-download: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/paula/Desktop/PBD/PositionBasedDynamics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Performing download step (git clone) for 'Ext_GenericParameters'"
	cd /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src && /usr/bin/cmake -P /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/tmp/Ext_GenericParameters-gitclone.cmake
	cd /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src && /usr/bin/cmake -E touch /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-download

GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-install: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/paula/Desktop/PBD/PositionBasedDynamics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Performing install step for 'Ext_GenericParameters'"
	cd /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-build && $(MAKE) install
	cd /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-build && /usr/bin/cmake -E touch /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-install

GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/paula/Desktop/PBD/PositionBasedDynamics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Creating directories for 'Ext_GenericParameters'"
	/usr/bin/cmake -E make_directory /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters
	/usr/bin/cmake -E make_directory /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-build
	/usr/bin/cmake -E make_directory /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters
	/usr/bin/cmake -E make_directory /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/tmp
	/usr/bin/cmake -E make_directory /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-stamp
	/usr/bin/cmake -E make_directory /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src
	/usr/bin/cmake -E make_directory /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-stamp
	/usr/bin/cmake -E touch /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-mkdir

GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-patch: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-update
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/paula/Desktop/PBD/PositionBasedDynamics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No patch step for 'Ext_GenericParameters'"
	/usr/bin/cmake -E echo_append
	/usr/bin/cmake -E touch /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-patch

GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-update: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/paula/Desktop/PBD/PositionBasedDynamics/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Performing update step for 'Ext_GenericParameters'"
	cd /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/src/Ext_GenericParameters && /usr/bin/cmake -P /home/paula/Desktop/PBD/PositionBasedDynamics/build/GenericParameters/tmp/Ext_GenericParameters-gitupdate.cmake

Ext_GenericParameters: CMakeFiles/Ext_GenericParameters
Ext_GenericParameters: CMakeFiles/Ext_GenericParameters-complete
Ext_GenericParameters: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-build
Ext_GenericParameters: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-configure
Ext_GenericParameters: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-download
Ext_GenericParameters: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-install
Ext_GenericParameters: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-mkdir
Ext_GenericParameters: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-patch
Ext_GenericParameters: GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-update
Ext_GenericParameters: CMakeFiles/Ext_GenericParameters.dir/build.make
.PHONY : Ext_GenericParameters

# Rule to build all files generated by this target.
CMakeFiles/Ext_GenericParameters.dir/build: Ext_GenericParameters
.PHONY : CMakeFiles/Ext_GenericParameters.dir/build

CMakeFiles/Ext_GenericParameters.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Ext_GenericParameters.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Ext_GenericParameters.dir/clean

CMakeFiles/Ext_GenericParameters.dir/depend:
	cd /home/paula/Desktop/PBD/PositionBasedDynamics/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/paula/Desktop/PBD/PositionBasedDynamics /home/paula/Desktop/PBD/PositionBasedDynamics /home/paula/Desktop/PBD/PositionBasedDynamics/build /home/paula/Desktop/PBD/PositionBasedDynamics/build /home/paula/Desktop/PBD/PositionBasedDynamics/build/CMakeFiles/Ext_GenericParameters.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Ext_GenericParameters.dir/depend


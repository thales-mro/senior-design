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
CMAKE_SOURCE_DIR = /home/thales/TCC/v-rep-remote-api/source

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/thales/TCC/v-rep-remote-api/build

# Include any dependencies generated for this target.
include CMakeFiles/remote_api.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/remote_api.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/remote_api.dir/flags.make

CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o: CMakeFiles/remote_api.dir/flags.make
CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o: /home/thales/TCC/v-rep-remote-api/source/src/v_rep/extApi.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/thales/TCC/v-rep-remote-api/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o   -c /home/thales/TCC/v-rep-remote-api/source/src/v_rep/extApi.c

CMakeFiles/remote_api.dir/src/v_rep/extApi.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/remote_api.dir/src/v_rep/extApi.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/thales/TCC/v-rep-remote-api/source/src/v_rep/extApi.c > CMakeFiles/remote_api.dir/src/v_rep/extApi.c.i

CMakeFiles/remote_api.dir/src/v_rep/extApi.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/remote_api.dir/src/v_rep/extApi.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/thales/TCC/v-rep-remote-api/source/src/v_rep/extApi.c -o CMakeFiles/remote_api.dir/src/v_rep/extApi.c.s

CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o.requires:

.PHONY : CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o.requires

CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o.provides: CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o.requires
	$(MAKE) -f CMakeFiles/remote_api.dir/build.make CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o.provides.build
.PHONY : CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o.provides

CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o.provides.build: CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o


CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o: CMakeFiles/remote_api.dir/flags.make
CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o: /home/thales/TCC/v-rep-remote-api/source/src/v_rep/extApiPlatform.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/thales/TCC/v-rep-remote-api/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o   -c /home/thales/TCC/v-rep-remote-api/source/src/v_rep/extApiPlatform.c

CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/thales/TCC/v-rep-remote-api/source/src/v_rep/extApiPlatform.c > CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.i

CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/thales/TCC/v-rep-remote-api/source/src/v_rep/extApiPlatform.c -o CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.s

CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o.requires:

.PHONY : CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o.requires

CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o.provides: CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o.requires
	$(MAKE) -f CMakeFiles/remote_api.dir/build.make CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o.provides.build
.PHONY : CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o.provides

CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o.provides.build: CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o


# Object files for target remote_api
remote_api_OBJECTS = \
"CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o" \
"CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o"

# External object files for target remote_api
remote_api_EXTERNAL_OBJECTS =

lib/libremote_api.a: CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o
lib/libremote_api.a: CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o
lib/libremote_api.a: CMakeFiles/remote_api.dir/build.make
lib/libremote_api.a: CMakeFiles/remote_api.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/thales/TCC/v-rep-remote-api/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C static library lib/libremote_api.a"
	$(CMAKE_COMMAND) -P CMakeFiles/remote_api.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/remote_api.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/remote_api.dir/build: lib/libremote_api.a

.PHONY : CMakeFiles/remote_api.dir/build

CMakeFiles/remote_api.dir/requires: CMakeFiles/remote_api.dir/src/v_rep/extApi.c.o.requires
CMakeFiles/remote_api.dir/requires: CMakeFiles/remote_api.dir/src/v_rep/extApiPlatform.c.o.requires

.PHONY : CMakeFiles/remote_api.dir/requires

CMakeFiles/remote_api.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/remote_api.dir/cmake_clean.cmake
.PHONY : CMakeFiles/remote_api.dir/clean

CMakeFiles/remote_api.dir/depend:
	cd /home/thales/TCC/v-rep-remote-api/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/thales/TCC/v-rep-remote-api/source /home/thales/TCC/v-rep-remote-api/source /home/thales/TCC/v-rep-remote-api/build /home/thales/TCC/v-rep-remote-api/build /home/thales/TCC/v-rep-remote-api/build/CMakeFiles/remote_api.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/remote_api.dir/depend


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
CMAKE_SOURCE_DIR = /home/ivan/git/bm_optitrack/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ivan/git/bm_optitrack/build

# Include any dependencies generated for this target.
include CMakeFiles/blabbermouth.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/blabbermouth.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/blabbermouth.dir/flags.make

CMakeFiles/blabbermouth.dir/bm_datastream.c.o: CMakeFiles/blabbermouth.dir/flags.make
CMakeFiles/blabbermouth.dir/bm_datastream.c.o: /home/ivan/git/bm_optitrack/src/bm_datastream.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivan/git/bm_optitrack/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/blabbermouth.dir/bm_datastream.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/blabbermouth.dir/bm_datastream.c.o   -c /home/ivan/git/bm_optitrack/src/bm_datastream.c

CMakeFiles/blabbermouth.dir/bm_datastream.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/blabbermouth.dir/bm_datastream.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ivan/git/bm_optitrack/src/bm_datastream.c > CMakeFiles/blabbermouth.dir/bm_datastream.c.i

CMakeFiles/blabbermouth.dir/bm_datastream.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/blabbermouth.dir/bm_datastream.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ivan/git/bm_optitrack/src/bm_datastream.c -o CMakeFiles/blabbermouth.dir/bm_datastream.c.s

CMakeFiles/blabbermouth.dir/bm_datastream.c.o.requires:

.PHONY : CMakeFiles/blabbermouth.dir/bm_datastream.c.o.requires

CMakeFiles/blabbermouth.dir/bm_datastream.c.o.provides: CMakeFiles/blabbermouth.dir/bm_datastream.c.o.requires
	$(MAKE) -f CMakeFiles/blabbermouth.dir/build.make CMakeFiles/blabbermouth.dir/bm_datastream.c.o.provides.build
.PHONY : CMakeFiles/blabbermouth.dir/bm_datastream.c.o.provides

CMakeFiles/blabbermouth.dir/bm_datastream.c.o.provides.build: CMakeFiles/blabbermouth.dir/bm_datastream.c.o


CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o: CMakeFiles/blabbermouth.dir/flags.make
CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o: /home/ivan/git/bm_optitrack/src/bm_tcp_datastream.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivan/git/bm_optitrack/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o   -c /home/ivan/git/bm_optitrack/src/bm_tcp_datastream.c

CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ivan/git/bm_optitrack/src/bm_tcp_datastream.c > CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.i

CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ivan/git/bm_optitrack/src/bm_tcp_datastream.c -o CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.s

CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o.requires:

.PHONY : CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o.requires

CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o.provides: CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o.requires
	$(MAKE) -f CMakeFiles/blabbermouth.dir/build.make CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o.provides.build
.PHONY : CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o.provides

CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o.provides.build: CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o


CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o: CMakeFiles/blabbermouth.dir/flags.make
CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o: /home/ivan/git/bm_optitrack/src/bm_bt_datastream.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivan/git/bm_optitrack/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o   -c /home/ivan/git/bm_optitrack/src/bm_bt_datastream.c

CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ivan/git/bm_optitrack/src/bm_bt_datastream.c > CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.i

CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ivan/git/bm_optitrack/src/bm_bt_datastream.c -o CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.s

CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o.requires:

.PHONY : CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o.requires

CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o.provides: CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o.requires
	$(MAKE) -f CMakeFiles/blabbermouth.dir/build.make CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o.provides.build
.PHONY : CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o.provides

CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o.provides.build: CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o


CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o: CMakeFiles/blabbermouth.dir/flags.make
CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o: /home/ivan/git/bm_optitrack/src/bm_dispatcher.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivan/git/bm_optitrack/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o   -c /home/ivan/git/bm_optitrack/src/bm_dispatcher.c

CMakeFiles/blabbermouth.dir/bm_dispatcher.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/blabbermouth.dir/bm_dispatcher.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ivan/git/bm_optitrack/src/bm_dispatcher.c > CMakeFiles/blabbermouth.dir/bm_dispatcher.c.i

CMakeFiles/blabbermouth.dir/bm_dispatcher.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/blabbermouth.dir/bm_dispatcher.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ivan/git/bm_optitrack/src/bm_dispatcher.c -o CMakeFiles/blabbermouth.dir/bm_dispatcher.c.s

CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o.requires:

.PHONY : CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o.requires

CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o.provides: CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o.requires
	$(MAKE) -f CMakeFiles/blabbermouth.dir/build.make CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o.provides.build
.PHONY : CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o.provides

CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o.provides.build: CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o


CMakeFiles/blabbermouth.dir/server.c.o: CMakeFiles/blabbermouth.dir/flags.make
CMakeFiles/blabbermouth.dir/server.c.o: /home/ivan/git/bm_optitrack/src/server.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivan/git/bm_optitrack/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/blabbermouth.dir/server.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/blabbermouth.dir/server.c.o   -c /home/ivan/git/bm_optitrack/src/server.c

CMakeFiles/blabbermouth.dir/server.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/blabbermouth.dir/server.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ivan/git/bm_optitrack/src/server.c > CMakeFiles/blabbermouth.dir/server.c.i

CMakeFiles/blabbermouth.dir/server.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/blabbermouth.dir/server.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ivan/git/bm_optitrack/src/server.c -o CMakeFiles/blabbermouth.dir/server.c.s

CMakeFiles/blabbermouth.dir/server.c.o.requires:

.PHONY : CMakeFiles/blabbermouth.dir/server.c.o.requires

CMakeFiles/blabbermouth.dir/server.c.o.provides: CMakeFiles/blabbermouth.dir/server.c.o.requires
	$(MAKE) -f CMakeFiles/blabbermouth.dir/build.make CMakeFiles/blabbermouth.dir/server.c.o.provides.build
.PHONY : CMakeFiles/blabbermouth.dir/server.c.o.provides

CMakeFiles/blabbermouth.dir/server.c.o.provides.build: CMakeFiles/blabbermouth.dir/server.c.o


CMakeFiles/blabbermouth.dir/main.c.o: CMakeFiles/blabbermouth.dir/flags.make
CMakeFiles/blabbermouth.dir/main.c.o: /home/ivan/git/bm_optitrack/src/main.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivan/git/bm_optitrack/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/blabbermouth.dir/main.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/blabbermouth.dir/main.c.o   -c /home/ivan/git/bm_optitrack/src/main.c

CMakeFiles/blabbermouth.dir/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/blabbermouth.dir/main.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ivan/git/bm_optitrack/src/main.c > CMakeFiles/blabbermouth.dir/main.c.i

CMakeFiles/blabbermouth.dir/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/blabbermouth.dir/main.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ivan/git/bm_optitrack/src/main.c -o CMakeFiles/blabbermouth.dir/main.c.s

CMakeFiles/blabbermouth.dir/main.c.o.requires:

.PHONY : CMakeFiles/blabbermouth.dir/main.c.o.requires

CMakeFiles/blabbermouth.dir/main.c.o.provides: CMakeFiles/blabbermouth.dir/main.c.o.requires
	$(MAKE) -f CMakeFiles/blabbermouth.dir/build.make CMakeFiles/blabbermouth.dir/main.c.o.provides.build
.PHONY : CMakeFiles/blabbermouth.dir/main.c.o.provides

CMakeFiles/blabbermouth.dir/main.c.o.provides.build: CMakeFiles/blabbermouth.dir/main.c.o


# Object files for target blabbermouth
blabbermouth_OBJECTS = \
"CMakeFiles/blabbermouth.dir/bm_datastream.c.o" \
"CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o" \
"CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o" \
"CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o" \
"CMakeFiles/blabbermouth.dir/server.c.o" \
"CMakeFiles/blabbermouth.dir/main.c.o"

# External object files for target blabbermouth
blabbermouth_EXTERNAL_OBJECTS =

blabbermouth: CMakeFiles/blabbermouth.dir/bm_datastream.c.o
blabbermouth: CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o
blabbermouth: CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o
blabbermouth: CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o
blabbermouth: CMakeFiles/blabbermouth.dir/server.c.o
blabbermouth: CMakeFiles/blabbermouth.dir/main.c.o
blabbermouth: CMakeFiles/blabbermouth.dir/build.make
blabbermouth: /usr/lib/x86_64-linux-gnu/libpthread.so
blabbermouth: /usr/lib/x86_64-linux-gnu/libbluetooth.so
blabbermouth: CMakeFiles/blabbermouth.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ivan/git/bm_optitrack/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking C executable blabbermouth"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/blabbermouth.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/blabbermouth.dir/build: blabbermouth

.PHONY : CMakeFiles/blabbermouth.dir/build

CMakeFiles/blabbermouth.dir/requires: CMakeFiles/blabbermouth.dir/bm_datastream.c.o.requires
CMakeFiles/blabbermouth.dir/requires: CMakeFiles/blabbermouth.dir/bm_tcp_datastream.c.o.requires
CMakeFiles/blabbermouth.dir/requires: CMakeFiles/blabbermouth.dir/bm_bt_datastream.c.o.requires
CMakeFiles/blabbermouth.dir/requires: CMakeFiles/blabbermouth.dir/bm_dispatcher.c.o.requires
CMakeFiles/blabbermouth.dir/requires: CMakeFiles/blabbermouth.dir/server.c.o.requires
CMakeFiles/blabbermouth.dir/requires: CMakeFiles/blabbermouth.dir/main.c.o.requires

.PHONY : CMakeFiles/blabbermouth.dir/requires

CMakeFiles/blabbermouth.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/blabbermouth.dir/cmake_clean.cmake
.PHONY : CMakeFiles/blabbermouth.dir/clean

CMakeFiles/blabbermouth.dir/depend:
	cd /home/ivan/git/bm_optitrack/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ivan/git/bm_optitrack/src /home/ivan/git/bm_optitrack/src /home/ivan/git/bm_optitrack/build /home/ivan/git/bm_optitrack/build /home/ivan/git/bm_optitrack/build/CMakeFiles/blabbermouth.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/blabbermouth.dir/depend


# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.17

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

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = D:\TOOLS\cmake\bin\cmake.exe

# The command to remove a file.
RM = D:\TOOLS\cmake\bin\cmake.exe -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:\M1\mujoco\vctest

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\M1\mujoco\vctest\build

# Include any dependencies generated for this target.
include CMakeFiles/doublependulum.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/doublependulum.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/doublependulum.dir/flags.make

CMakeFiles/doublependulum.dir/src/doublependulum.cpp.obj: CMakeFiles/doublependulum.dir/flags.make
CMakeFiles/doublependulum.dir/src/doublependulum.cpp.obj: CMakeFiles/doublependulum.dir/includes_CXX.rsp
CMakeFiles/doublependulum.dir/src/doublependulum.cpp.obj: ../src/doublependulum.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=D:\M1\mujoco\vctest\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/doublependulum.dir/src/doublependulum.cpp.obj"
	D:\TOOLS\mingw\x86_64-13.1.0-release-win32-seh-msvcrt-rt_v11-rev1\mingw64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\doublependulum.dir\src\doublependulum.cpp.obj -c D:\M1\mujoco\vctest\src\doublependulum.cpp

CMakeFiles/doublependulum.dir/src/doublependulum.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/doublependulum.dir/src/doublependulum.cpp.i"
	D:\TOOLS\mingw\x86_64-13.1.0-release-win32-seh-msvcrt-rt_v11-rev1\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\M1\mujoco\vctest\src\doublependulum.cpp > CMakeFiles\doublependulum.dir\src\doublependulum.cpp.i

CMakeFiles/doublependulum.dir/src/doublependulum.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/doublependulum.dir/src/doublependulum.cpp.s"
	D:\TOOLS\mingw\x86_64-13.1.0-release-win32-seh-msvcrt-rt_v11-rev1\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\M1\mujoco\vctest\src\doublependulum.cpp -o CMakeFiles\doublependulum.dir\src\doublependulum.cpp.s

# Object files for target doublependulum
doublependulum_OBJECTS = \
"CMakeFiles/doublependulum.dir/src/doublependulum.cpp.obj"

# External object files for target doublependulum
doublependulum_EXTERNAL_OBJECTS =

doublependulum.exe: CMakeFiles/doublependulum.dir/src/doublependulum.cpp.obj
doublependulum.exe: CMakeFiles/doublependulum.dir/build.make
doublependulum.exe: ../lib/mujoco.lib
doublependulum.exe: ../lib/glfw3.dll
doublependulum.exe: CMakeFiles/doublependulum.dir/linklibs.rsp
doublependulum.exe: CMakeFiles/doublependulum.dir/objects1.rsp
doublependulum.exe: CMakeFiles/doublependulum.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=D:\M1\mujoco\vctest\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable doublependulum.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\doublependulum.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/doublependulum.dir/build: doublependulum.exe

.PHONY : CMakeFiles/doublependulum.dir/build

CMakeFiles/doublependulum.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\doublependulum.dir\cmake_clean.cmake
.PHONY : CMakeFiles/doublependulum.dir/clean

CMakeFiles/doublependulum.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\M1\mujoco\vctest D:\M1\mujoco\vctest D:\M1\mujoco\vctest\build D:\M1\mujoco\vctest\build D:\M1\mujoco\vctest\build\CMakeFiles\doublependulum.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/doublependulum.dir/depend


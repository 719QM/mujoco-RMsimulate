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
include CMakeFiles/doublependulum_ik.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/doublependulum_ik.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/doublependulum_ik.dir/flags.make

CMakeFiles/doublependulum_ik.dir/src/doublependulum_ik.cpp.obj: CMakeFiles/doublependulum_ik.dir/flags.make
CMakeFiles/doublependulum_ik.dir/src/doublependulum_ik.cpp.obj: CMakeFiles/doublependulum_ik.dir/includes_CXX.rsp
CMakeFiles/doublependulum_ik.dir/src/doublependulum_ik.cpp.obj: ../src/doublependulum_ik.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=D:\M1\mujoco\vctest\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/doublependulum_ik.dir/src/doublependulum_ik.cpp.obj"
	D:\TOOLS\mingw\x86_64-13.1.0-release-win32-seh-msvcrt-rt_v11-rev1\mingw64\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\doublependulum_ik.dir\src\doublependulum_ik.cpp.obj -c D:\M1\mujoco\vctest\src\doublependulum_ik.cpp

CMakeFiles/doublependulum_ik.dir/src/doublependulum_ik.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/doublependulum_ik.dir/src/doublependulum_ik.cpp.i"
	D:\TOOLS\mingw\x86_64-13.1.0-release-win32-seh-msvcrt-rt_v11-rev1\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E D:\M1\mujoco\vctest\src\doublependulum_ik.cpp > CMakeFiles\doublependulum_ik.dir\src\doublependulum_ik.cpp.i

CMakeFiles/doublependulum_ik.dir/src/doublependulum_ik.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/doublependulum_ik.dir/src/doublependulum_ik.cpp.s"
	D:\TOOLS\mingw\x86_64-13.1.0-release-win32-seh-msvcrt-rt_v11-rev1\mingw64\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S D:\M1\mujoco\vctest\src\doublependulum_ik.cpp -o CMakeFiles\doublependulum_ik.dir\src\doublependulum_ik.cpp.s

# Object files for target doublependulum_ik
doublependulum_ik_OBJECTS = \
"CMakeFiles/doublependulum_ik.dir/src/doublependulum_ik.cpp.obj"

# External object files for target doublependulum_ik
doublependulum_ik_EXTERNAL_OBJECTS =

doublependulum_ik.exe: CMakeFiles/doublependulum_ik.dir/src/doublependulum_ik.cpp.obj
doublependulum_ik.exe: CMakeFiles/doublependulum_ik.dir/build.make
doublependulum_ik.exe: ../lib/mujoco.lib
doublependulum_ik.exe: ../lib/glfw3.dll
doublependulum_ik.exe: CMakeFiles/doublependulum_ik.dir/linklibs.rsp
doublependulum_ik.exe: CMakeFiles/doublependulum_ik.dir/objects1.rsp
doublependulum_ik.exe: CMakeFiles/doublependulum_ik.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=D:\M1\mujoco\vctest\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable doublependulum_ik.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\doublependulum_ik.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/doublependulum_ik.dir/build: doublependulum_ik.exe

.PHONY : CMakeFiles/doublependulum_ik.dir/build

CMakeFiles/doublependulum_ik.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\doublependulum_ik.dir\cmake_clean.cmake
.PHONY : CMakeFiles/doublependulum_ik.dir/clean

CMakeFiles/doublependulum_ik.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\M1\mujoco\vctest D:\M1\mujoco\vctest D:\M1\mujoco\vctest\build D:\M1\mujoco\vctest\build D:\M1\mujoco\vctest\build\CMakeFiles\doublependulum_ik.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/doublependulum_ik.dir/depend


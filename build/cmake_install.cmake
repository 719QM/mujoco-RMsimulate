# Install script for directory: D:/M1/mujoco/vctest

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "D:/M1/mujoco/vctest/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "D:/M1/mujoco/vctest/install/mujoco.dll;D:/M1/mujoco/vctest/install/glfw3.dll;D:/M1/mujoco/vctest/install/ball.xml")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "D:/M1/mujoco/vctest/install" TYPE FILE FILES
    "D:/M1/mujoco/vctest/lib/mujoco.dll"
    "D:/M1/mujoco/vctest/lib/glfw3.dll"
    "D:/M1/mujoco/vctest/model/ball.xml"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "D:/M1/mujoco/vctest/install/MuJoCoDemo_1.exe")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "D:/M1/mujoco/vctest/install" TYPE EXECUTABLE FILES "D:/M1/mujoco/vctest/build/MuJoCoDemo_1.exe")
  if(EXISTS "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/MuJoCoDemo_1.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/MuJoCoDemo_1.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "D:/TOOLS/mingw/x86_64-13.1.0-release-win32-seh-msvcrt-rt_v11-rev1/mingw64/bin/strip.exe" "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/MuJoCoDemo_1.exe")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "D:/M1/mujoco/vctest/install/pendulum.exe")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "D:/M1/mujoco/vctest/install" TYPE EXECUTABLE FILES "D:/M1/mujoco/vctest/build/pendulum.exe")
  if(EXISTS "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/pendulum.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/pendulum.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "D:/TOOLS/mingw/x86_64-13.1.0-release-win32-seh-msvcrt-rt_v11-rev1/mingw64/bin/strip.exe" "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/pendulum.exe")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "D:/M1/mujoco/vctest/install/doublependulum.exe")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "D:/M1/mujoco/vctest/install" TYPE EXECUTABLE FILES "D:/M1/mujoco/vctest/build/doublependulum.exe")
  if(EXISTS "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/doublependulum.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/doublependulum.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "D:/TOOLS/mingw/x86_64-13.1.0-release-win32-seh-msvcrt-rt_v11-rev1/mingw64/bin/strip.exe" "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/doublependulum.exe")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "D:/M1/mujoco/vctest/install/doublependulum_fsm.exe")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "D:/M1/mujoco/vctest/install" TYPE EXECUTABLE FILES "D:/M1/mujoco/vctest/build/doublependulum_fsm.exe")
  if(EXISTS "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/doublependulum_fsm.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/doublependulum_fsm.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "D:/TOOLS/mingw/x86_64-13.1.0-release-win32-seh-msvcrt-rt_v11-rev1/mingw64/bin/strip.exe" "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/doublependulum_fsm.exe")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "D:/M1/mujoco/vctest/install/doublependulum_ik.exe")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "D:/M1/mujoco/vctest/install" TYPE EXECUTABLE FILES "D:/M1/mujoco/vctest/build/doublependulum_ik.exe")
  if(EXISTS "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/doublependulum_ik.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/doublependulum_ik.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "D:/TOOLS/mingw/x86_64-13.1.0-release-win32-seh-msvcrt-rt_v11-rev1/mingw64/bin/strip.exe" "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/doublependulum_ik.exe")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "D:/M1/mujoco/vctest/install/hybrid_pendulum.exe")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "D:/M1/mujoco/vctest/install" TYPE EXECUTABLE FILES "D:/M1/mujoco/vctest/build/hybrid_pendulum.exe")
  if(EXISTS "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/hybrid_pendulum.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/hybrid_pendulum.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "D:/TOOLS/mingw/x86_64-13.1.0-release-win32-seh-msvcrt-rt_v11-rev1/mingw64/bin/strip.exe" "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/hybrid_pendulum.exe")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "D:/M1/mujoco/vctest/install/arm.exe")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "D:/M1/mujoco/vctest/install" TYPE EXECUTABLE FILES "D:/M1/mujoco/vctest/build/arm.exe")
  if(EXISTS "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/arm.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/arm.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "D:/TOOLS/mingw/x86_64-13.1.0-release-win32-seh-msvcrt-rt_v11-rev1/mingw64/bin/strip.exe" "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/arm.exe")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "D:/M1/mujoco/vctest/install/ik.exe")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "D:/M1/mujoco/vctest/install" TYPE EXECUTABLE FILES "D:/M1/mujoco/vctest/build/ik.exe")
  if(EXISTS "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/ik.exe" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/ik.exe")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "D:/TOOLS/mingw/x86_64-13.1.0-release-win32-seh-msvcrt-rt_v11-rev1/mingw64/bin/strip.exe" "$ENV{DESTDIR}/D:/M1/mujoco/vctest/install/ik.exe")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "D:/M1/mujoco/vctest/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")

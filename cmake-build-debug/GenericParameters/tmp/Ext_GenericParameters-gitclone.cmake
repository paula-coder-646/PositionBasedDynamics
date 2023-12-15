# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

if(EXISTS "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitclone-lastrun.txt" AND EXISTS "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitinfo.txt" AND
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitclone-lastrun.txt" IS_NEWER_THAN "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitinfo.txt")
  message(STATUS
    "Avoiding repeated git clone, stamp file is up to date: "
    "'/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitclone-lastrun.txt'"
  )
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: '/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "/usr/bin/git"
            clone --no-checkout --config "advice.detachedHead=false" "https://github.com/InteractiveComputerGraphics/GenericParameters.git" "Ext_GenericParameters"
    WORKING_DIRECTORY "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src"
    RESULT_VARIABLE error_code
  )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once: ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/InteractiveComputerGraphics/GenericParameters.git'")
endif()

execute_process(
  COMMAND "/usr/bin/git"
          checkout "a4e2744eea526270cfe38b826440d09f66473316" --
  WORKING_DIRECTORY "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: 'a4e2744eea526270cfe38b826440d09f66473316'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "/usr/bin/git" 
            submodule update --recursive --init 
    WORKING_DIRECTORY "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters"
    RESULT_VARIABLE error_code
  )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: '/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitinfo.txt" "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
)
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: '/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters-stamp/Ext_GenericParameters-gitclone-lastrun.txt'")
endif()

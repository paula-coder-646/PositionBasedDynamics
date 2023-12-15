# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters"
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters-build"
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters"
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/tmp"
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters-stamp"
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src"
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/GenericParameters/src/Ext_GenericParameters-stamp${cfgdir}") # cfgdir has leading slash
endif()

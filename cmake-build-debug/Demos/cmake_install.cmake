# Install script for directory: /home/paula/Desktop/PBD/PositionBasedDynamics/Demos

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/Demos" TYPE DIRECTORY FILES "/home/paula/Desktop/PBD/PositionBasedDynamics/Demos/./Common" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/Demos/BarDemo/cmake_install.cmake")
  include("/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/Demos/ClothDemo/cmake_install.cmake")
  include("/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/Demos/CosseratRodsDemo/cmake_install.cmake")
  include("/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/Demos/CouplingDemos/cmake_install.cmake")
  include("/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/Demos/DistanceFieldDemos/cmake_install.cmake")
  include("/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/Demos/FluidDemo/cmake_install.cmake")
  include("/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/Demos/GenericConstraintsDemos/cmake_install.cmake")
  include("/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/Demos/PositionBasedElasticRodsDemo/cmake_install.cmake")
  include("/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/Demos/RigidBodyDemos/cmake_install.cmake")
  include("/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/Demos/SceneLoaderDemo/cmake_install.cmake")
  include("/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/Demos/StiffRodsDemos/cmake_install.cmake")

endif()


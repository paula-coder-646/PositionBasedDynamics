# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/extern/Discregrid/src/Ext_Discregrid"
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/extern/Discregrid/src/Ext_Discregrid-build"
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/Discregrid"
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/extern/Discregrid/tmp"
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/extern/Discregrid/src/Ext_Discregrid-stamp"
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/extern/Discregrid/src"
  "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/extern/Discregrid/src/Ext_Discregrid-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/extern/Discregrid/src/Ext_Discregrid-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/paula/Desktop/PBD/PositionBasedDynamics/cmake-build-debug/extern/Discregrid/src/Ext_Discregrid-stamp${cfgdir}") # cfgdir has leading slash
endif()

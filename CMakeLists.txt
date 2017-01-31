# Project name
PROJECT(DoorDetector)

# CMake required version must be >= 2.6
cmake_minimum_required(VERSION 2.6)

IF(CMAKE_VERSION VERSION_LESS "3.1")
   SET(CMAKE_CXX_FLAGS "-std=c++0x")
   ELSE()
     SET(CMAKE_CXX_STANDARD 11)
ENDIF()

# Find DisCODe installation
FIND_PACKAGE(DisCODe REQUIRED)
FIND_PACKAGE( OpenCV REQUIRED )

# Use CMake macros provided with DisCODe
set(CMAKE_MODULE_PATH ${DisCODe_DIR} ${CMAKE_MODULE_PATH})
include(DisCODeMacros)

# Set default install prefix to dist folder
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX
      ${CMAKE_SOURCE_DIR}/dist CACHE PATH "${CMAKE_PROJECT_NAME} install prefix" FORCE
  )
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

ADD_SUBDIRECTORY(src)

REBUILD_DCL_CACHE()

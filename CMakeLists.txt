# Copyright (C) LAAS-CNRS
# Author : Elie MOUSSY
#
# This file is part of calibrationPTZ.
# calibrationPTZ is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/lapack.cmake)
INCLUDE(cmake/boost.cmake)

SET(PROJECT_NAME calibrationPTZ)
SET(PROJECT_DESCRIPTION "Calibrate a camera PTZ")
SET(PROJECT_URL "http://github.com/elie-moussy/calibrationPTZ")
SET(PROJECT_VERSION 1.0)

SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE ON)

ADD_SUBDIRECTORY(include)

SETUP_PROJECT()
SEARCH_FOR_LAPACK()
SEARCH_FOR_BOOST()

ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.2.0")
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})
ADD_REQUIRED_DEPENDENCY("controlPTZ")
INCLUDE_DIRECTORIES(${controlPTZ_INCLUDE_DIRS})
ADD_REQUIRED_DEPENDENCY("opencv >= 2.4.2")
INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIRS})
FIND_LIBRARY(LEVMAR_LIB NAMES levmar PATHS ${CMAKE_INSTALL_LIBDIR})
FIND_LIBRARY(CONTROLPTZ_LIB NAMES controlPTZ PATHS ${CMAKE_INSTALL_LIBDIR})
FIND_LIBRARY(OPENCV_NONFREE_LIB NAMES opencv_nonfree PATHS ${CMAKE_INSTALL_LIBDIR})

PKG_CONFIG_APPEND_LIBS("calibrationPTZ")

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

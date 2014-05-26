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

SETUP_PROJECT()
SEARCH_FOR_LAPACK()
SEARCH_FOR_BOOST()

ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.2.0")
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

PKG_CONFIG_APPEND_LIBS("calibration")

ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()

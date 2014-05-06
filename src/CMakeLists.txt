# Copyright (C) LAAS-CNRS
# Author : Elie MOUSSY
#
# This file is part of math_tools.
# math_tools is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

SET(SOURCES
	matrix.cpp )

ADD_LIBRARY(calibration SHARED ${SOURCES})
INSTALL(TARGETS calibration DESTINATION ${CMAKE_INSTALL_LIBDIR})

# Copyright (C) LAAS-CNRS
# Author : Elie MOUSSY
#
# This file is part of calibrationPTZ.
# calibrationPTZ is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

SET(SOURCES
	matrix.cpp
	Calibration.cpp)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

TARGET_LINK_LIBRARIES( ${PROJECT_NAME}
  ${CONTROLPTZ_LIB}

  # OpenCV libs
  opencv_core
  opencv_highgui
  opencv_imgproc
  opencv_video
  opencv_legacy
  opencv_contrib
  opencv_features2d
  opencv_calib3d
  ${OPENCV_NONFREE_LIB}
  opencv_flann

  # other libs
  ${LEVMAR_LIB}
  curl
  lapack
  blas)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_LIBDIR})

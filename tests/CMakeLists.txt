# Copyright (C) LAAS-CNRS
# Author : Elie MOUSSY
#
# This file is part of calibration.
# calibration is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# Add Boost path
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Make Boost.Test generates the main function in test cases
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

MACRO(CALIBRATION_TEST NAME)
  ADD_EXECUTABLE(${NAME} ${NAME}.cpp ${ARGN})
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  # Link against calibration.
  TARGET_LINK_LIBRARIES(${NAME} ${PROJECT_NAME})

  # Link against LAPACK.
  TARGET_LINK_LIBRARIES(${NAME} ${LAPACK_LIBRARIES})

  # Link against Boost.
  TARGET_LINK_LIBRARIES(${NAME} ${Boost_LIBRARIES})
  
  ADD_DEPENDENCIES(${NAME} ${PROJECT_NAME})

Endmacro(CALIBRATION_TEST)

# pseudo inverse test
CALIBRATION_TEST(testpseudoinverse)

# polyeig1 test
CALIBRATION_TEST(testpolyeig1)

# polyeig2 test
CALIBRATION_TEST(testpolyeig2)

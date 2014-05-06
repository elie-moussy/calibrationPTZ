# Copyright (C) LAAS-CNRS
# Author : Elie MOUSSY
#
# This file is part of math_tools.
# math_tools is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

SET(NEWHEADERS
	matrix.h)

INSTALL(FILES ${NEWHEADERS}
	DESTINATION include/calibration
	PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE )

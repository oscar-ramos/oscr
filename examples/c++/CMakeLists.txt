# Copyright 2017, Oscar Ramos
#
# This file is part of oscr.
#
# oscr is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# oscr is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License for more details. You should
# have received a copy of the GNU Lesser General Public License along
# with oscr. If not, see <http://www.gnu.org/licenses/>.

INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
  ${RBDL_LIBRARY_DIRS}
  ${PINOCCHIO_LIBRARY_DIRS}
  ${QPOASES_LIBRARY_DIRS}
)

SET(OSCR_LIBRARIES
  oscr
  ${RBDL_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
  ${QPOASES_LIBRARIES}
)
# Copy urdf models to test
FILE(GLOB URDF_MODELS
  ${CMAKE_SOURCE_DIR}/examples/models/ff6dof.urdf
)
FILE(COPY ${URDF_MODELS} DESTINATION ${CMAKE_BINARY_DIR}/examples/models)

ADD_EXECUTABLE(ff6dofModel ff6dof-model.cpp)
TARGET_LINK_LIBRARIES(ff6dofModel ${OSCR_LIBRARIES})

ADD_EXECUTABLE(ff6dofKineControl ff6dof-kine-control.cpp)
TARGET_LINK_LIBRARIES(ff6dofKineControl ${OSCR_LIBRARIES})



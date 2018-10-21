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


# Add the include directory to the include paths
INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIRS}
  ${PINOCCHIO_INCLUDE_DIRS}
  ${QPOASES_INCLUDE_DIRS}
  )

LINK_DIRECTORIES(
  ${RBDL_LIBRARY_DIRS}
  ${PINOCCHIO_LIBRARY_DIRS}
  ${QPOASES_LIBRARY_DIRS}
)

# Only for RBDL (RBDL does not provide link numbers or joint limits, therefore
# direct use of URDF is needed ... baaaad)
INCLUDE_DIRECTORIES(${URDF_INCLUDE_DIRS})
LINK_DIRECTORIES(${URDF_LIBRARY_DIRS})

# Library containing oscr
ADD_LIBRARY(oscr SHARED
  math-utils.cpp
  robot-model.cpp
  robot-model-rbdl.cpp
  robot-model-pin.cpp
  robot-model-ff6dof.cpp
  model-utils.cpp
  vector-logger.cpp
  kine-task.cpp
  kine-task-pose.cpp
  kine-task-com.cpp
  kine-task-posture.cpp
  osik-solver.cpp
  osik-solver-NS.cpp
  osik-solver-WQP.cpp
  osik-solver-HQP.cpp
  )

SET_TARGET_PROPERTIES (oscr PROPERTIES
  VERSION ${OSCR_VERSION}
  SOVERSION ${OSCR_SO_VERSION}
  )

# # Needed when qpoases does not have pkg config
# IF(NOT ${QPOASES_FOUND})
#   TARGET_LINK_LIBRARIES(oscr ${qpOASES_LIBRARIES})
# ENDIF()

FILE(GLOB HEADERS_OSCR  ${CMAKE_SOURCE_DIR}/include/oscr/*.hpp)
FILE(GLOB HEADERS_TOOLS ${CMAKE_SOURCE_DIR}/include/oscr/tools/*.hpp)
FILE(GLOB HEADERS_MODEL ${CMAKE_SOURCE_DIR}/include/oscr/model/*.hpp)
FILE(GLOB HEADERS_IK    ${CMAKE_SOURCE_DIR}/include/oscr/ik/*.hpp)

# Installation targets
INSTALL(TARGETS oscr DESTINATION lib)
INSTALL(FILES ${HEADERS_OSCR}  DESTINATION include/oscr)
INSTALL(FILES ${HEADERS_TOOLS} DESTINATION include/oscr/tools)
INSTALL(FILES ${HEADERS_MODEL} DESTINATION include/oscr/model)
INSTALL(FILES ${HEADERS_IK} DESTINATION include/oscr/ik)


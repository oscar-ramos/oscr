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

FIND_PACKAGE(PythonInterp REQUIRED)
FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})
INCLUDE(SwigPython)

# Relative path where python modules will be installed
EXECUTE_PROCESS(
  COMMAND "${PYTHON_EXECUTABLE}" "-c"
  "import sys, os; print(os.sep.join(['lib', 'python' + sys.version[:3], 'site-packages']))"
  OUTPUT_VARIABLE PYTHON_SITELIB
  ERROR_QUIET)
STRING(REPLACE "\n" "" PYTHON_SITELIB "${PYTHON_SITELIB}")
SET(PYTHON_INSTALL_DIR ${PYTHON_SITELIB}/oscr)

LINK_DIRECTORIES(
  ${QPOASES_LIBRARY_DIRS}
  ${RBDL_LIBRARY_DIRS}
  ${PINOCCHIO_LIBRARY_DIRS})

# Needed for all the modules since they all link to oscr
SET(OSCR_LIBRARIES
  oscr
  ${QPOASES_LIBRARIES}
  ${RBDL_LIBRARIES}
  ${PINOCCHIO_LIBRARIES}
  )

# ADD_SWIG_PYTHON_MODULE(
#   mathUtilsPython mathUtils.i
#   CPLUSPLUS
#   INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS}
#   LINK_LIBRARIES ${OSCR_LIBRARIES}
#   SWIG_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/modules/swig-eigen
#   DESTINATION ${PYTHON_INSTALL_DIR}
#   )

# ADD_SWIG_PYTHON_MODULE(
#   robotModelRbdlPython robotModelRbdl.i
#   CPLUSPLUS
#   INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS} ${RBDL_INCLUDE_DIRS}
#   LINK_LIBRARIES ${OSCR_LIBRARIES}
#   SWIG_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/modules/swig-eigen
#   DESTINATION ${PYTHON_INSTALL_DIR}
#   )

# ADD_SWIG_PYTHON_MODULE(
#   robotModelPinPython robotModelPin.i
#   CPLUSPLUS
#   INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS} ${PINOCCHIO_INCLUDE_DIRS}
#   LINK_LIBRARIES ${OSCR_LIBRARIES}
#   SWIG_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/modules/swig-eigen
#   DESTINATION ${PYTHON_INSTALL_DIR}
#   )

# ADD_SWIG_PYTHON_MODULE(
#   kineTaskPosePython kineTaskPose.i
#   CPLUSPLUS
#   INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS} ${PINOCCHIO_INCLUDE_DIRS}
#   LINK_LIBRARIES ${OSCR_LIBRARIES}
#   SWIG_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/modules/swig-eigen
#   DESTINATION ${PYTHON_INSTALL_DIR}
#   )

ADD_SWIG_PYTHON_MODULE(
  oscrPython oscr.i
  CPLUSPLUS
  INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS} ${PINOCCHIO_INCLUDE_DIRS}
  LINK_LIBRARIES ${OSCR_LIBRARIES}
  SWIG_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/modules/swig-eigen
  DESTINATION ${PYTHON_INSTALL_DIR}
  )

INSTALL(FILES
  __init__.py
  oscr/utils.py
  oscr/robot_base.py
  oscr/ros_markers.py
  oscr/ros_pubs.py
  oscr/ros_logs.py
  oscr/ros_robot.py
  oscr/ros_kine_tasks.py
  oscr/ros_kine_sim.py
  DESTINATION ${PYTHON_INSTALL_DIR})


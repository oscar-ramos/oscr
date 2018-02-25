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

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(OSCR CXX)

# Project version
SET (OSCR_VERSION_MAJOR 0)
SET (OSCR_VERSION_MINOR 0)
SET (OSCR_VERSION_PATCH 1)
SET (OSCR_VERSION
  ${OSCR_VERSION_MAJOR}.${OSCR_VERSION_MINOR}.${OSCR_VERSION_PATCH})
SET (OSCR_SO_VERSION
  ${OSCR_VERSION_MAJOR}.${OSCR_VERSION_MINOR}.${OSCR_VERSION_PATCH})
SET(PROJECT_VERSION ${OSCR_VERSION})

# Path for additional CMake modules
SET(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/swig-eigen/cmake
  )

FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(EIGEN3 eigen3>=3.2 REQUIRED)
PKG_CHECK_MODULES(RBDL rbdl>=2.5 REQUIRED)
PKG_CHECK_MODULES(PINOCCHIO pinocchio REQUIRED)
PKG_CHECK_MODULES(QPOASES qpoases>=3.2 REQUIRED)
PKG_CHECK_MODULES(URDFDOM urdfdom>=0.2 REQUIRED)
# Try to find qpoases if there is no pkg-config for it
IF(NOT ${QPOASES_FOUND})
  FIND_PACKAGE(QPOASES)
ENDIF()

# Set default build type to 'Release' if none is specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  MESSAGE(STATUS "Using build type: 'Release' (none was specified).")
  SET(CMAKE_BUILD_TYPE RELEASE CACHE STRING "Choose the type of build." FORCE)
ENDIF()

# Special care for Eigen>3.2.10
IF(EIGEN3_FOUND)
  IF(${EIGEN3_VERSION} VERSION_GREATER "3.2.10")
    ADD_DEFINITIONS(-DEIGEN3_FUTURE)
    SET(EIGEN3_FUTURE TRUE)
    SET(_PKG_CONFIG_CFLAGS "${_PKG_CONFIG_CFLAGS} -DEIGEN3_FUTURE" CACHE INTERNAL "")
    IF(${EIGEN3_VERSION} VERSION_LESS "3.3.0")
      ADD_DEFINITIONS(-DEIGEN3_BETA_3_2_9x)
      SET(EIGEN3_FUTURE TRUE)
      SET(_PKG_CONFIG_CFLAGS "${_PKG_CONFIG_CFLAGS} -DEIGEN3_BETA_3_2_9x" CACHE INTERNAL "")
    ENDIF(${EIGEN3_VERSION} VERSION_LESS "3.3.0")
  ELSE(${EIGEN3_VERSION} VERSION_GREATER "3.2.10")
    SET(EIGEN3_FUTURE FALSE)
  ENDIF(${EIGEN3_VERSION} VERSION_GREATER "3.2.10")
ENDIF(EIGEN3_FOUND)

# Special care for urdfdom versions
IF(URDFDOM_FOUND)
  IF(${URDFDOM_VERSION} VERSION_LESS "0.3.0")
    ADD_DEFINITIONS(-DURDFDOM_COLLISION_WITH_GROUP_NAME)
    SET(_PKG_CONFIG_CFLAGS "${_PKG_CONFIG_CFLAGS} -DURDFDOM_COLLISION_WITH_GROUP_NAME" CACHE INTERNAL "")
  ENDIF(${URDFDOM_VERSION} VERSION_LESS "0.3.0")
  # Types from version 0.4.0
  IF(NOT ${URDFDOM_VERSION} VERSION_LESS "0.4.0")
    ADD_DEFINITIONS(-DURDFDOM_TYPEDEF_SHARED_PTR)
    SET(_PKG_CONFIG_CFLAGS "${_PKG_CONFIG_CFLAGS} -DURDFDOM_TYPEDEF_SHARED_PTR" CACHE INTERNAL "")
  ENDIF(NOT ${URDFDOM_VERSION} VERSION_LESS "0.4.0")
  # From version 1.0.0, std::shared_ptr appears
  IF(${URDFDOM_VERSION} VERSION_GREATER "0.4.2")
    ADD_DEFINITIONS(-DURDFDOM_USE_STD_SHARED_PTR)
    SET(_PKG_CONFIG_CFLAGS "${_PKG_CONFIG_CFLAGS} -DURDFDOM_USE_STD_SHARED_PTR" CACHE INTERNAL "")
  ENDIF(${URDFDOM_VERSION} VERSION_GREATER "0.4.2")
ENDIF(URDFDOM_FOUND)

# OSCR Library
ADD_SUBDIRECTORY(src)

# OSCR unit tests
#ADD_SUBDIRECTORY(unitTests)

# OSCR c++ example
ADD_SUBDIRECTORY(examples/c++)

# Python modules
ADD_SUBDIRECTORY(modules)

# Set pkg-config file
SET(CMAKE_INSTALL_FULL_LIBDIR ${CMAKE_INSTALL_PREFIX}/lib)
SET(CMAKE_INSTALL_FULL_INCLUDEDIR ${CMAKE_INSTALL_PREFIX}/include)
CONFIGURE_FILE (
  ${CMAKE_SOURCE_DIR}/oscr.pc.cmake
  ${CMAKE_BINARY_DIR}/oscr.pc @ONLY
  )
# Install pkg-config file
INSTALL(
  FILES  ${CMAKE_BINARY_DIR}/oscr.pc
  DESTINATION lib/pkgconfig
  )


# Doxygen for documentation
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
  CONFIGURE_FILE(
    ${CMAKE_SOURCE_DIR}/doc/Doxyfile.in
    ${CMAKE_BINARY_DIR}/doc/Doxyfile @ONLY)
  ADD_CUSTOM_TARGET(doc
    ${DOXYGEN_EXECUTABLE}
    ${CMAKE_BINARY_DIR}/doc/Doxyfile
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/doc
    COMMENT "Generating Doxygen documentation" VERBATIM
    )
  # # Install documentation
  # # TODO: from scratch, make then make install gives
  # # error due to documentation (make, make doc, make install: no
  # # error). Solve this
  # INSTALL(DIRECTORY ${CMAKE_BINARY_DIR}/doc/doxygen-html
  #   DESTINATION share/doc/oscr)
ENDIF(DOXYGEN_FOUND)

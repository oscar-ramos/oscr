# FindQPOASES
# -----------
#
# Try to find the qpOASES library when there is no pkg-config for the package
# (the default case in the original qpOASES project).
#
# It is assumed that qpOASES has been installed in a linux platform (using
# cmake install) to a given prefix. Please specify the installation prefix with
# the environmental variable QPOASES_INSTALL_PREFIX as:
#
#    export QPOASES_INSTALL_PREFIX=prefix_to_qpoasis_installation
#
# This variable will be used as a hint to find the installed header files and
# libraries. Please note that OSCR uses shared libraries, then, you should
# change STATIC (the default) to SHARED in qpOASES' CMakeLists.txt.
#
# Once done, the following variables are defined:
#
#    QPOASES_FOUND         - System has qpOASES
#    QPOASES_INCLUDE_DIRS  - qpOASES include directory
#    QPOASES_LIBRARIES     - qpOASES libraries
#
# NOTE: In the qpoases packed with the OSCR-DEPS package there is no need to
# use this file since a pkg-config file for qpoases was added

INCLUDE(FindPackageHandleStandardArgs)

FIND_PATH(qpOASES_INCLUDEDIR
  NAMES qpOASES.hpp
  HINTS "${qpOASES_INSTALL_PREFIX}"
  ENV qpOASES_INSTALL_PREFIX
  PATH_SUFFIXES include)
FIND_LIBRARY(qpOASES_LIB
  NAMES qpOASES
  HINTS "${qpOASES_INSTALL_PREFIX}"
  ENV qpOASES_INSTALL_PREFIX
  PATH_SUFFIXES lib
  libs)

SET(QPOASES_INCLUDE_DIRS ${qpOASES_INCLUDEDIR})
SET(QPOASES_LIBRARIES ${qpOASES_LIB})

FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  qpOASES DEFAULT_MSG
  QPOASES_LIBRARIES QPOASES_INCLUDE_DIRS)

SET(QPOASES_FOUND ${QPOASES_FOUND})

IF(NOT QPOASES_FOUND)
  MESSAGE(FATAL_ERROR "Could not find qpOASES in your system. Install it and/or set the environmental variable qpOASES_INSTALL_PREFIX to the installation path of qpOASES.")
ENDIF()

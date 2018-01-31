# Copyright (c) 2015, Robin Deits
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holder nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


INCLUDE(CMakeParseArguments)

FUNCTION(add_swig_python_module target i_file)
  # Parse arguments and make sure we got the required ones
  SET(options CPLUSPLUS)
  SET(oneValueArgs)
  SET(multiValueArgs INCLUDE_DIRS LINK_LIBRARIES SWIG_INCLUDE_DIRS DESTINATION)
  CMAKE_PARSE_ARGUMENTS(
    swigpy 
    "${options}"
    "${oneValueArgs}" 
    "${multiValueArgs}" 
    ${ARGN} 
    )
  IF (NOT target)
    MESSAGE(FATAL_ERROR "Error using add_swig_python_module: Please provide a unique cmake target name as the first argument")
  ENDIF()
  IF (NOT i_file)
    MESSAGE(FATAL_ERROR "Error using add_swig_python_module: Please provide the path to your .i file as the second argument")
  ENDIF()
  
  # Find python and get its version number
  FIND_PACKAGE(PythonInterp REQUIRED)
  SET(PYVERSION "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
  
  # Do not override an explicit choice of Python libraries.
  IF(NOT PYTHON_LIBRARY AND NOT PYTHON_INCLUDE_DIR)
    EXECUTE_PROCESS(
      COMMAND "${PYTHON_EXECUTABLE}" -c "import sys; print(sys.exec_prefix)"
      RESULT_VARIABLE PYTHON_EXEC_PREFIX_RESULT
      OUTPUT_VARIABLE PYTHON_EXEC_PREFIX
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    
    IF(PYTHON_EXEC_PREFIX_RESULT EQUAL 0)
      LIST(APPEND CMAKE_PREFIX_PATH ${PYTHON_EXEC_PREFIX})
    ELSE()
      MESSAGE(WARNING "Could NOT determine Python sys.exec_prefix")
    ENDIF()
  ENDIF()
    
  # PYTHON_LIBRARY and PYTHON_INCLUDE_DIR (singular) are (cached) inputs.
  # PYTHON_LIBRARIES and PYTHON_INCLUDE_DIRS (plural) are outputs.
  FIND_PACKAGE(PythonLibs MODULE REQUIRED)
  INCLUDE_DIRECTORIES( ${PYTHON_INCLUDE_DIRS} )
  
  # Load the swig macros
  IF (NOT SWIG_EXECUTABLE)
    FIND_PACKAGE(SWIG REQUIRED)
  ENDIF()
  
  # Find the numpy header paths and include them. This calls the
  # FindNumPy.cmake file included in this repo.
  FIND_PACKAGE(NumPy REQUIRED)
  INCLUDE_DIRECTORIES(${NUMPY_INCLUDE_DIRS})
  
  # Include source directories that swig will need to find c++ header files
  FOREACH(dir IN LISTS swigpy_INCLUDE_DIRS)
    INCLUDE_DIRECTORIES(${dir})
  ENDFOREACH(dir)
  
  # Tell SWIG that we're compiling a c++ (not c) file, and tell it to use
  # python3 if appropriate.
  IF (swigpy_CPLUSPLUS)
    SET(CPLUSPLUS ON)
  ELSE()
    SET(CPLUSPLUS OFF)
  ENDIF()
  SET_SOURCE_FILES_PROPERTIES(${i_file} PROPERTIES CPLUSPLUS ${CPLUSPLUS})
  IF (PYTHON_VERSION_MAJOR GREATER 2)
    SET_PROPERTY(SOURCE 
      ${i_file}
      APPEND PROPERTY 
      SWIG_FLAGS "-py3" "-DSWIGPYTHON3")
  ENDIF()
  
  # Tell swig to also look for .i interface files in these folders
  FOREACH(dir IN LISTS swigpy_SWIG_INCLUDE_DIRS)
    SET(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-I${dir}")
  ENDFOREACH(dir)
  
  # Use "modern" python classes to resolve
  # https://github.com/casadi/casadi/issues/1364
  SET(CMAKE_SWIG_FLAGS ${CMAKE_SWIG_FLAGS} "-modern")
  
  # Tell swig to build python bindings for our target library and link them
  # against the C++ library.
  SWIG_ADD_MODULE(${target} python ${i_file})
  SWIG_LINK_LIBRARIES(${target} ${swigpy_LINK_LIBRARIES} ${PYTHON_LIBRARIES})
  
  # Make sure the resulting library has the correct name, even if the cmake
  # target has a different name
  SET_TARGET_PROPERTIES(
    ${SWIG_MODULE_${target}_REAL_NAME} 
    PROPERTIES
    OUTPUT_NAME
    _${SWIG_GET_EXTRA_OUTPUT_FILES_module_basename}
    )
  
  # Automatically install to the correct subfolder if the swig module has a
  # "package" declared
  IF (swig_package_name)
    STRING(REGEX REPLACE "\\." "/" swigpy_package_path ${swig_package_name})
  ENDIF()
  
  FOREACH(dir IN LISTS swigpy_DESTINATION)
    INSTALL(
      TARGETS
      ${SWIG_MODULE_${target}_REAL_NAME}
      DESTINATION 
      ${dir}/${swigpy_package_path}
      )
    FOREACH(file IN LISTS swig_extra_generated_files)
      INSTALL(FILES ${file} DESTINATION ${dir}/${swigpy_package_path})
    ENDFOREACH(file)
  ENDFOREACH(dir)
  
  # Clean up
  SET_SOURCE_FILES_PROPERTIES(${i_file} PROPERTIES SWIG_FLAGS "")

ENDFUNCTION()

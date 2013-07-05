#
#  Copyright 2013 CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)
INCLUDE(cmake/eigen.cmake)

SET(PROJECT_NAME viaopt)
SET(PROJECT_DESCRIPTION "...")
SET(PROJECT_URL "http://github.com/laas/viaopt")

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE true)

project(${PROJECT_NAME})

# C++ version 4.7 is less permissive than older versions.
#  To avoid the created errors, the flag -fpermissive is added.
# TODO: remove this patch
IF(UNIX)
  exec_program(${CMAKE_CXX_COMPILER}
    ARGS ${CMAKE_CXX_COMPILER_ARG1} -dumpversion
    OUTPUT_VARIABLE _cxx_COMPILER_VERSION
  )
  string(REGEX REPLACE "([0-9])\\.([0-9])(\\.[0-9])?" "\\1\\2"
    _cxx_COMPILER_VERSION ${_cxx_COMPILER_VERSION})

  IF(${_cxx_COMPILER_VERSION} GREATER 46 )
    SET(ADD_CXX_PATCH TRUE)
    ADD_DEFINITIONS("-fpermissive")
  ENDIF(${_cxx_COMPILER_VERSION} GREATER 46 )
ENDIF(UNIX)

SETUP_PROJECT()

# Search for dependencies.
SEARCH_FOR_EIGEN()

# Handle OS specificities 
INCLUDE(CheckIncludeFiles)
CHECK_INCLUDE_FILES("sys/time.h" SYS_TIME_H)
IF(SYS_TIME_H)
ADD_DEFINITIONS("-DHAVE_SYS_TIME_H")
ENDIF(SYS_TIME_H)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(unitTesting)

IF(ADD_CXX_PATCH)
  PKG_CONFIG_APPEND_CFLAGS("-fpermissive")
ENDIF(ADD_CXX_PATCH)
PKG_CONFIG_APPEND_CFLAGS(${_Eigen_CFLAGS})
PKG_CONFIG_APPEND_LIBS("viaopt")

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()


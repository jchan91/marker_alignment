CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(CMAKE_LEGACY_CYGWIN_WIN32 0)

PROJECT(marker_alignment)

# The library prefix
SET(LIB_PREFIX marker_alignment_)

SET(MARKER_ALIGNMENT_C_FLAGS)
SET(MARKER_ALIGNMENT_CXX_FLAGS)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# postfix, based on type
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")
SET(CMAKE_RELWITHDEBINFO_POSTFIX "_rd" CACHE STRING "postfix applied to release-with-debug-information libraries")
SET(CMAKE_MINSIZEREL_POSTFIX "_s" CACHE STRING "postfix applied to minimium-size-build libraries")

# work out the postfix; required where we use OUTPUT_NAME
IF(CMAKE_BUILD_TYPE MATCHES Release)
  SET(EXE_POSTFIX)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Debug)
  SET(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  SET(EXE_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES MinSizeRel)
  SET(EXE_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

# Allow the developer to select if Dynamic or Static libraries are built
OPTION (BUILD_SHARED_LIBS "Build Shared Libraries" ON)
SET (MARKER_ALIGNMENT_LIB_TYPE STATIC)
IF (BUILD_SHARED_LIBS)
  SET (MARKER_ALIGNMENT_LIB_TYPE SHARED)
ENDIF()

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)

# On the Mac platform, configure the RPATH as per the install, to
# avoid the problem of loading both the built and installed versions
# of the shared targets
IF(APPLE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
  SET(CMAKE_INSTALL_RPATH "")
ENDIF(APPLE)

# Set the output directory for the build executables and libraries
SET(MARKER_ALIGNMENT_SOURCE_DIR ${CMAKE_SOURCE_DIR})
SET(MARKER_ALIGNMENT_RUNTIME_OUTPUT_DIRECTORY ${MARKER_ALIGNMENT_SOURCE_DIR}/bin CACHE PATH "Target for the binaries")

IF(WIN32)
  SET(MARKER_ALIGNMENT_LIBRARY_OUTPUT_DIRECTORY ${MARKER_ALIGNMENT_SOURCE_DIR}/bin CACHE PATH "Target for the libraries")
ELSE(WIN32)
  SET(MARKER_ALIGNMENT_LIBRARY_OUTPUT_DIRECTORY ${MARKER_ALIGNMENT_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")
ENDIF(WIN32)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${MARKER_ALIGNMENT_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${MARKER_ALIGNMENT_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${MARKER_ALIGNMENT_RUNTIME_OUTPUT_DIRECTORY})

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
LIST(APPEND CMAKE_MODULE_PATH ${MARKER_ALIGNMENT_SOURCE_DIR}/cmake_modules)

# Detect OS and define macros appropriately
IF(WIN32)
  ADD_DEFINITIONS(-DWINDOWS)
  MESSAGE(STATUS "Compiling on Windows")
  IF(MSVC)
    MESSAGE(STATUS "Compiling with MSVC")
    ADD_DEFINITIONS(-DNOMINMAX)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    # SSE2 optimizations
    ADD_DEFINITIONS("/arch:SSE2")
    IF (BUILD_SHARED_LIBS)
      # disable warning on missing DLL interfaces
      ADD_DEFINITIONS("/wd4251")
    ENDIF()
  ENDIF(MSVC)
ELSEIF(CYGWIN)
  MESSAGE(STATUS "Compiling on Cygwin")
  ADD_DEFINITIONS(-DCYGWIN)
ELSEIF(APPLE)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on OSX")
ELSEIF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ENDIF(WIN32)

# detect Android Cross Compiler
# based on android-cmake which sets the variable ANDROID for us
IF(ANDROID)
  ADD_DEFINITIONS(-DANDROID)
  MESSAGE(STATUS "Cross compiling for Android")
ENDIF()

# For building the CHOLMOD / CSPARSE solvers
#FIND_PACKAGE(Cholmod)
#FIND_PACKAGE(BLAS)
#FIND_PACKAGE(LAPACK)
#IF(CHOLMOD_FOUND AND BLAS_FOUND AND LAPACK_FOUND)
#  MESSAGE(STATUS "Found CHOLMOD and its dependencies")
#  SET(CHOLMOD_FOUND TRUE)
#ELSE()
#  SET(CHOLMOD_FOUND FALSE)
#ENDIF()

#FIND_PACKAGE(CSparse)
#IF(CSPARSE_FOUND)
#  SET(BUILD_CSPARSE OFF CACHE BOOL "Build local CSparse library")
#ELSE(CSPARSE_FOUND)
#  SET(BUILD_CSPARSE ON CACHE BOOL "Build local CSparse library")
#  IF(BUILD_CSPARSE)
#    SET(CSPARSE_FOUND TRUE)
#  ENDIF()
#ENDIF(CSPARSE_FOUND)
#OPTION(BUILD_LGPL_SHARED_LIBS "Build LGPL Code as Shared Libraries (LGPL Code)" ON)
#SET (G2O_LGPL_LIB_TYPE STATIC)
#IF (BUILD_LGPL_SHARED_LIBS)
#  SET (G2O_LGPL_LIB_TYPE SHARED)
#ELSE()
#  MESSAGE(STATUS "Building LGPL code as static library (affects license of the binary)")
#ENDIF()

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
FIND_PACKAGE(OpenMP)
SET(MARKER_ALIGNMENT_USE_OPENMP OFF CACHE BOOL "Build g2d with OpenMP support (EXPERIMENTAL)")
IF(OPENMP_FOUND AND MARKER_ALIGNMENT_USE_OPENMP)
  SET (MARKER_ALIGNMENT_OPENMP 1)
  SET(MARKER_ALIGNMENT_C_FLAGS "${MARKER_ALIGNMENT_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(MARKER_ALIGNMENT_CXX_FLAGS "${MARKER_ALIGNMENT_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Compiling with OpenMP support")
ENDIF(OPENMP_FOUND AND MARKER_ALIGNMENT_USE_OPENMP)

# OpenGL is used in the draw actions for the different types, as well
# as for creating the GUI itself
#FIND_PACKAGE(OpenGL)
#SET(G2O_USE_OPENGL ON CACHE BOOL "Build g2o with OpenGL support for visualization")
#IF (OPENGL_FOUND AND G2O_USE_OPENGL)
#  SET (G2O_HAVE_OPENGL 1)
#  INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
#  MESSAGE(STATUS "Compiling with OpenGL support")
#ENDIF()

# For building the GUI
#FIND_PACKAGE(QGLViewer)

# For non-linear least squares solver
FIND_PACKAGE(Ceres REQUIRED)

# For general CV stuff
FIND_PACKAGE(OpenCV REQUIRED)

# For PCL to help visualize stuff
find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# For VTK to help visualize stuff
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# Boost for lots of nice stuff
find_package(Boost 1.36.0 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# shall we build the core apps using the library
SET(MARKER_ALIGNMENT_BUILD_APPS ON CACHE BOOL "Build MARKER_ALIGNMENT apps")
IF(MARKER_ALIGNMENT_BUILD_APPS)
  MESSAGE(STATUS "Compiling MARKER_ALIGNMENT apps")
ENDIF(MARKER_ALIGNMENT_BUILD_APPS)

include(CMakeDependentOption)
CMAKE_DEPENDENT_OPTION(MARKER_ALIGNMENT_BUILD_LINKED_APPS "Build apps linked with the libraries (no plugin system)" OFF
  "MARKER_ALIGNMENT_BUILD_APPS" OFF)

# Compiler specific options for gcc
IF(CMAKE_COMPILER_IS_GNUCXX)
  OPTION (BUILD_WITH_MARCH_NATIVE "Build with \"-march native\"" ON)
  MESSAGE(STATUS "Compiling with GCC")
  
  IF(NOT ${ARCH} MATCHES arm)
    # Generic settings for optimisation
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -msse4.2")
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse4.2")
  ENDIF()
  # switch off optimization for debug builds
  SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
  SET(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

  # OS X
  #IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    #SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}") 
    #SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}") 
  #ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # Linux
  IF(BUILD_WITH_MARCH_NATIVE AND NOT ${ARCH} MATCHES arm AND ${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
  ENDIF()
  # activate warnings !!!
  SET(MARKER_ALIGNMENT_C_FLAGS "${MARKER_ALIGNMENT_C_FLAGS} -Wall -W")
  SET(MARKER_ALIGNMENT_CXX_FLAGS "${MARKER_ALIGNMENT_CXX_FLAGS} -Wall -W")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

IF(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  MESSAGE(STATUS "Compiling with Clang")

  # activate all warnings
  #SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Weverything")
  #SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Weverything")
  SET(MARKER_ALIGNMENT_C_FLAGS "${MARKER_ALIGNMENT_C_FLAGS} -Wall")
  SET(MARKER_ALIGNMENT_CXX_FLAGS "${MARKER_ALIGNMENT_CXX_FLAGS} -Wall")
  #SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -stdlib=libc++")
ENDIF()

# C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

# Find Eigen3
SET(EIGEN3_INCLUDE_DIR ${MARKER_ALIGNMENT_EIGEN3_INCLUDE})
FIND_PACKAGE(Eigen3)
IF(EIGEN3_FOUND)
  SET(MARKER_ALIGNMENT_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
ELSE(EIGEN3_FOUND)
  SET(MARKER_ALIGNMENT_EIGEN3_INCLUDE "" CACHE PATH "Directory of Eigen3")
ENDIF(EIGEN3_FOUND)

# Set up the top-level include directories
INCLUDE_DIRECTORIES(${MARKER_ALIGNMENT_SOURCE_DIR} ${MARKER_ALIGNMENT_EIGEN3_INCLUDE})

# Generate config.h
#SET(G2O_OPENGL_FOUND ${OPENGL_FOUND})
#SET(G2O_HAVE_CHOLMOD ${CHOLMOD_FOUND})
#SET(G2O_HAVE_CSPARSE ${CSPARSE_FOUND})
SET(MARKER_ALIGNMENT_SHARED_LIBS ${BUILD_SHARED_LIBS})
#SET(G2O_LGPL_SHARED_LIBS ${BUILD_LGPL_SHARED_LIBS})
SET(MARKER_ALIGNMENT_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
#configure_file(config.h.in ${PROJECT_BINARY_DIR}/marker_alignment/config.h)
include_directories(${PROJECT_BINARY_DIR})
#INSTALL(FILES ${PROJECT_BINARY_DIR}/marker_alignment/config.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/marker_alignment)

# Include the subdirectories
ADD_SUBDIRECTORY(marker_alignment)

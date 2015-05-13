# ----------------------------------------------------------------------------------------------------
# kth-rgbd: Visual SLAM from RGB-D data
# Copyright (C) 2011  Virgile Högman
# ----------------------------------------------------------------------------------------------------
project(kth-rgbd)

cmake_minimum_required(VERSION 2.4.6)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
  cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

# ----------------------------------------------------------------------------------------------------
# !!! PATH TO CUSTOMIZE !!!
# ----------------------------------------------------------------------------------------------------
set(sourcepath ~/projects/slam/)
set(g2o_SOURCE_DIR ${sourcepath}/g2o/trunk/)	# http://openslam.org/g2o.html
set(OpenSIFT_DIR ${sourcepath}/opensift/)			# http://robwhess.github.com/opensift/

# ----------------------------------------------------------------------------------------------------
# if errors also check include directories for OpenNI and Eigen3 below.
# ----------------------------------------------------------------------------------------------------
find_package(Boost 1.46 REQUIRED COMPONENTS filesystem)
find_package(PCL 1.7 REQUIRED COMPONENTS common io registration)
find_package(OpenCV 2.4 REQUIRED)

#message("DEBUG: ---CHECK PATH---")
#message("DEBUG: PCL_INCLUDE=${PCL_INCLUDE_DIRS}")
#message("DEBUG: PCL_LIBS=${PCL_LIBRARIES}")
#message("DEBUG: OPENCV_INC=${OpenCV_INCLUDE_DIRS}")
#message("DEBUG: OPENCV_LIBS=${OpenCV_LIBS}")

# ----------------------------------------------------------------------------------------------------
# includes
# ----------------------------------------------------------------------------------------------------
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenSIFT_DIR}/include)	
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/include/ni)		# OpenNI >= 1.5 
include_directories(/usr/include/eigen3)	# check this if errors!
include_directories(${g2o_SOURCE_DIR})	
include_directories(${g2o_SOURCE_DIR}/build/)	#for g2o/config.h

# ----------------------------------------------------------------------------------------------------
# libraries
# ----------------------------------------------------------------------------------------------------
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenSIFT_DIR}/lib)
link_directories(${g2o_SOURCE_DIR}/lib)

# ----------------------------------------------------------------------------------------------------
# sub-dependencies
# ----------------------------------------------------------------------------------------------------
# sub-dependencies of g2o
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${g2o_SOURCE_DIR}/cmake_modules)
find_package(CSparse)
#message("DEBUG: CSPARSE_INCLUDE_DIR=${CSPARSE_INCLUDE_DIR}")
#message("DEBUG: CSPARSE_LIBRARY=${CSPARSE_LIBRARY}")
if(NOT CSPARSE_FOUND)
	# g2o should build its own version in EXTERNAL
	message(STATUS "G2O - CSparse not found")	
	SET(G2O_BUILD_CSPARSE ON CACHE BOOL "Link with g2o local CSparse library")
else()
	message(STATUS "G2O - CSparse found")	
	SET(G2O_BUILD_CSPARSE OFF CACHE BOOL "Link with g2o local CSparse library")
endif()

if(G2O_BUILD_CSPARSE)
	# g2o should build its own version in EXTERNAL
	message(STATUS "G2O - Using G2O EXTERNAL CSparse")
	include_directories(${g2o_SOURCE_DIR}/EXTERNAL/csparse)
	set(CSPARSE_LIBRARY g2o_ext_csparse)
else()
	message(STATUS "G2O - Using System CSparse")
	include_directories(${CSPARSE_INCLUDE_DIR})
endif()

#Not tested with Suitesparse or Cholmod, dependencies with g2o unknown...
#find_package(SuiteSparse)
#set(SuiteSparse_LIBS cholmod amd colamd umfpack blas lapack suitesparseconfig)
#find_package(Cholmod)
#message("DEBUG: CHOLMOD_LIBRARIES=${CHOLMOD_LIBRARIES}")

set(g2o_LIBRARIES ${CSPARSE_LIBRARY} g2o_core g2o_solver_csparse g2o_csparse_extension g2o_solver_dense g2o_types_slam3d g2o_stuff)

# VTK2 is a sub-dependency of OpenSIFT
find_package (PkgConfig REQUIRED)
pkg_check_modules (GTK2 REQUIRED gtk+-2.0)
link_directories (${GTK2_LIBRARY_DIRS})
set(OpenSIFT_LIBRARIES opensift ${GTK2_LIBRARIES})
#message("DEBUG: GTK2_LIBRARIES=${GTK2_LIBRARIES}")
# if errors check manually with pkg-config --libs gtk+-2.0
# or force names with following line
# not clear which GTK and GDK libs to link with... this might depend on the distribution (Ubuntu, Arch, ...)
#set(OpenSIFT_LIBRARIES opensift gtk-x11-2.0 gdk-x11-2.0)

# ----------------------------------------------------------------------------------------------------
# sources, main and test
# ----------------------------------------------------------------------------------------------------
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(src)
add_subdirectory(test)

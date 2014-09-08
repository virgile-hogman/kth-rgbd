# ----------------------------------------------------------------------------------------------------
# kth-rgbd: Visual SLAM from RGB-D data
# Copyright (C) 2011  Virgile Högman
# ----------------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.4.6)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# ----------------------------------------------------------------------------------------------------
# !!! PATH TO CUSTOMIZE !!!
# ----------------------------------------------------------------------------------------------------
set(sourcepath ~/archive/projects/)
set(g2o_SOURCE_DIR ${sourcepath}/g2o/tags/before-github-sync)	# http://openslam.org/g2o.html
set(OpenSIFT_DIR ${sourcepath}/opensift)			# http://robwhess.github.com/opensift/
# ----------------------------------------------------------------------------------------------------
# if errors also check include directories for OpenNI and Eigen3 below.
# ----------------------------------------------------------------------------------------------------

find_package(Boost 1.46 COMPONENTS filesystem REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 2.4 REQUIRED)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${g2o_SOURCE_DIR}/cmake_modules)
#find_package(SuiteSparse)

message("---CHECK PATH---")
message("PCL_INCLUDE=${PCL_INCLUDE_DIRS}")
message("PCL_LIBS=${PCL_LIBRARIES}")
message("OPENCV_INC=${OpenCV_INCLUDE_DIRS}")
message("OPENCV_LIBS=${OpenCV_LIBS}")

# ----------------------------------------------------------------------------------------------------
# rgbd library
# ----------------------------------------------------------------------------------------------------
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenSIFT_DIR}/include)	
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/include/ni)		# OpenNI >= 1.5 
include_directories(/usr/include/eigen3)	# check this if errors!
include_directories(${g2o_SOURCE_DIR})	
include_directories(${g2o_SOURCE_DIR}/build/)	#for g2o/config.h
include_directories(${g2o_SOURCE_DIR}/EXTERNAL/csparse)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(RGBD_SOURCES_CPP FrameData.cpp Graph.cpp Sequence.cpp Config.cpp ConfigFile.cpp CameraDevice.cpp Matching.cpp PointCloud.cpp Display.cpp)

add_library(rgbd_slam ${RGBD_SOURCES_CPP}) 

# ----------------------------------------------------------------------------------------------------
# main executable
# ----------------------------------------------------------------------------------------------------
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenSIFT_DIR}/lib)
link_directories(${g2o_SOURCE_DIR}/lib)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set(SUITESPARSE_LIBRARIES cholmod amd colamd umfpack blas lapack suitesparseconfig)
#set(SUITESPARSE_LIBRARIES ${CHOLMOD_LIBRARIES})
set(g2o_LIBRARIES g2o_core g2o_math_groups g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_types_slam3d g2o_stuff g2o_ext_csparse ${SUITESPARSE_LIBRARIES})

add_executable(kth_rgbd rgbd.cpp)
target_link_libraries(kth_rgbd rgbd_slam ${Boost_LIBRARIES} opensift OpenNI ${OpenCV_LIBS} ${g2o_LIBRARIES} ${PCL_LIBRARIES}) 

# if errors with opensift and GTK it might be useful to link with GTK and GDK... this might depend on the distribution (Ubuntu / ArchLinux / ...)
#target_link_libraries(kth_rgbd rgbd_slam ${Boost_LIBRARIES} opensift gtk-x11-2.0 gdk-x11-2.0 OpenNI ${OpenCV_LIBS} ${g2o_LIBRARIES} ${PCL_LIBRARIES})
 

# ----------------------------------------------------------------------------------------------------
# kth-rgbd: Visual SLAM from RGB-D data
# Copyright (C) 2011  Virgile Högman
# ----------------------------------------------------------------------------------------------------
cmake_minimum_required(VERSION 2.4.6)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

find_package(PCL 1.2 REQUIRED)

# ----------------------------------------------------------------------------------------------------
# rgbd library
# ----------------------------------------------------------------------------------------------------
include_directories(${PCL_INCLUDE_DIRS})
include_directories(~/Projects/sift/include)	# Rob Hess SIFT library
include_directories(/usr/include/opencv-2.3.1/opencv) #SIFT library uses opencv root
include_directories(/usr/include/opencv-2.3.1/) #autoref in opencv to opencv2 folder
include_directories(/usr/include/ni)	#Open NI
include_directories(/usr/local/include/eigen3) #eigen3
include_directories(/usr/include/suitesparse)	#g2o cholmod
include_directories(~/Projects/g2o/trunk)	
include_directories(~/Projects/g2o/trunk/build/)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(RGBD_SOURCES_CPP FrameData.cpp Graph.cpp Sequence.cpp Config.cpp ConfigFile.cpp CameraDevice.cpp Matching.cpp PointCloud.cpp Display.cpp)

add_library(rgbd_slam ${RGBD_SOURCES_CPP}) 

# ----------------------------------------------------------------------------------------------------
# main executable
# ----------------------------------------------------------------------------------------------------
link_directories(${PCL_LIBRARY_DIRS})
link_directories(/usr/lib)
link_directories(~/Projects/sift/lib)
link_directories(~/Projects/g2o/trunk/lib)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(G2O_LIBRARIES g2o_core g2o_math_groups g2o_solver_cholmod g2o_types_slam3d)
set(OPENCV_LIBRARIES opencv_core opencv_highgui opencv_features2d)

add_executable(kth_rgbd rgbd.cpp)
target_link_libraries(kth_rgbd rgbd_slam boost_filesystem feat OpenNI ${OPENCV_LIBRARIES} ${G2O_LIBRARIES} ${PCL_LIBRARIES}) 

# ----------------------------------------------------------------------------------------------------
# kth-rgbd: Visual SLAM from RGB-D data
# Copyright (C) 2011  Virgile Högman
# ----------------------------------------------------------------------------------------------------

include_directories(../src)
include_directories(/usr/include/ni)	#Open NI
include_directories(/usr/local/include/eigen3) #eigen3

link_directories(/usr/lib)
link_directories(../lib)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(test_openni testOpenNI.cpp)
target_link_libraries(test_openni rgbd_slam boost_filesystem OpenNI) 

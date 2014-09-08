# ----------------------------------------------------------------------------------------------------
# kth-rgbd: Visual SLAM from RGB-D data
# Copyright (C) 2011  Virgile Högman
# ----------------------------------------------------------------------------------------------------
find_package(Boost 1.46 COMPONENTS filesystem REQUIRED)

include_directories(../src)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(/usr/include/ni)		#OpenNI >= 1.5 
include_directories(/usr/include/eigen3)	#eigen3

link_directories(/usr/lib)
link_directories(../lib)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

add_executable(test_openni testOpenNI.cpp)
target_link_libraries(test_openni rgbd_slam ${Boost_LIBRARIES} OpenNI) 

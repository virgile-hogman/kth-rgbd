cmake_minimum_required(VERSION 2.4.6)

include_directories(~/Projects/sift/include)
include_directories(/usr/include/opencv)
include_directories(/usr/include/ni)
#include_directories(/opt/ros/diamondback/stacks/perception_pcl/pcl/include/pcl/common/)
include_directories(/usr/local/include/eigen3)
include_directories(/usr/local/include/pcl-1.0/)

link_directories(/usr/lib)
link_directories(/usr/local/lib)
link_directories(~/Projects/sift/lib)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_executable(rgbd_slam rgbd_slam.cpp FrameData.cpp)
target_link_libraries(rgbd_slam feat OpenNI cv highgui boost_filesystem pcl_io pcl_filters) 


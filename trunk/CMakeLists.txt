cmake_minimum_required(VERSION 2.4.6)

include_directories(~/Projects/sift/include)
include_directories(/usr/include/opencv)
include_directories(/usr/include/ni)
#include_directories(/opt/ros/diamondback/stacks/perception_pcl/pcl/include/pcl/common/)
include_directories(/usr/local/include/eigen3)
include_directories(/usr/local/include/pcl-1.0/)

include_directories(/usr/include/suitesparse)
include_directories(~/Projects/g2o/trunk)
include_directories(~/Projects/g2o/trunk/build/)

link_directories(/usr/lib)
link_directories(/usr/local/lib)
link_directories(~/Projects/sift/lib)
link_directories(~/Projects/g2o/trunk/lib)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_executable(rgbd_slam rgbd_slam.cpp FrameData.cpp Graph.cpp Map.cpp Config.cpp CameraDevice.cpp Matching.cpp)
target_link_libraries(rgbd_slam feat OpenNI cv highgui boost_filesystem pcl_io g2o_core g2o_math_groups g2o_solver_cholmod g2o_types_slam3d) 


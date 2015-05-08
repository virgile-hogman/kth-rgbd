# ----------------------------------------------------------------------------------------------------
# kth-rgbd: Visual SLAM from RGB-D data
# Copyright (C) 2011  Virgile Högman
# ----------------------------------------------------------------------------------------------------

set(RGBD_SOURCES_CPP FrameData.cpp Graph.cpp Sequence.cpp Config.cpp ConfigFile.cpp CameraDevice.cpp Matching.cpp PointCloud.cpp Display.cpp)

add_library(rgbd_slam ${RGBD_SOURCES_CPP}) 

# ----------------------------------------------------------------------------------------------------
# main executable
# ----------------------------------------------------------------------------------------------------
add_executable(kth_rgbd main.cpp)
target_link_libraries(kth_rgbd rgbd_slam ${Boost_LIBRARIES} ${OpenSIFT_LIBRARIES} OpenNI ${OpenCV_LIBS} ${g2o_LIBRARIES} ${PCL_LIBRARIES}) 

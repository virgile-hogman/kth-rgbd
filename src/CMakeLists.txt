# ----------------------------------------------------------------------------------------------------
# kth-rgbd: Visual SLAM from RGB-D data
# Copyright (C) 2011  Virgile Högman
# ----------------------------------------------------------------------------------------------------
message("Main library")

set(RGBD_SOURCES_CPP FrameData.cpp Graph.cpp Sequence.cpp Config.cpp ConfigFile.cpp CameraDevice.cpp Matching.cpp PointCloud.cpp Display.cpp)

add_library(rgbd_slam ${RGBD_SOURCES_CPP}) 
target_link_libraries(rgbd_slam ${g2o_LIBRARIES} ${Boost_LIBRARIES} ${OpenSIFT_LIBRARIES} OpenNI ${OpenCV_LIBS} ${PCL_LIBRARIES})

# ----------------------------------------------------------------------------------------------------
# main executable
# ----------------------------------------------------------------------------------------------------
message("Main exe")
add_executable(kth_rgbd main.cpp)
target_link_libraries(kth_rgbd rgbd_slam) 

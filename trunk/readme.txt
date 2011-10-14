Requirements
------------

* Eigen3

* SIFT Library - Rob Hess
http://blogs.oregonstate.edu/hess/code/sift/

* OpenCV
http://opencv.willowgarage.com/

* OpenNI
http://www.openni.org/

* Point Cloud Libary
(cminpack, Flann, Eigen3, OpenNI)
pointclouds.org/

* Boost


Other
-----

* LibUSB

* GTK ?

Known bugs
----------

A memory leak still not found. Alloc can fail from a certain point (typically during the reconstruction when big amounts of memory are used for the pointclouds or during the matching).

--------------------------

OpenCV Error: Insufficient memory (Failed to allocate 1843200 bytes) in OutOfMemoryError, file /tmp/buildd/libopencv-2.3.1+svn6514+branch23/modules/core/src/alloc.cpp, line 52
terminate called after throwing an instance of 'cv::Exception'
  what():  /tmp/buildd/libopencv-2.3.1+svn6514+branch23/modules/core/src/alloc.cpp:52: error: (-4) Failed to allocate 1843200 bytes in function OutOfMemoryError

--------------------------

Frames 2734-2735:	 Extracting SIFT features... OpenCV Error: Null pointer () in cvCvtSeqToArray, file cxcore/cxdatastructs.cpp, line 548
terminate called after throwing an instance of 'cv::Exception'

this may be a pb in the SIFT library.

Requirements
------------

* Eigen3

* SIFT Library - Rob Hess
http://blogs.oregonstate.edu/hess/code/sift/

* OpenCV

* OpenNI

* Point Cloud Libary
(cminpack, Flann, Eigen3, OpenNI)

* Boost


Other
-----

* LibUSB

* GTK ?

Known bugs
----------

A memory leak still not found. Alloc can fail from a certain point (typically during the reconstruction when big amounts of memory are used for the pointclouds or during the matching).

Frames 2734-2735:	 Extracting SIFT features... OpenCV Error: Null pointer () in cvCvtSeqToArray, file cxcore/cxdatastructs.cpp, line 548
terminate called after throwing an instance of 'cv::Exception'



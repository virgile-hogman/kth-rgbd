// kth-rgbd: Visual SLAM from RGB-D data
// Copyright (C) 2011  Virgile Högman
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

Requirements
------------

* OpenCV
http://opencv.willowgarage.com/

* OpenNI
http://www.openni.org/

* Point Cloud Libary
(cminpack, Flann, Eigen3, OpenNI)
http://pointclouds.org/

* Eigen3
http://eigen.tuxfamily.org/

* SIFT Library - Rob Hess
http://blogs.oregonstate.edu/hess/code/sift/

* Boost Library
http://www.boost.org/

Other
-----

* LibUSB
* GTK ?
* subdependencies

Known bugs
----------

Frames 2734-2735:	 Extracting SIFT features... OpenCV Error: Null pointer () in cvCvtSeqToArray, file cxcore/cxdatastructs.cpp, line 548
terminate called after throwing an instance of 'cv::Exception'

this may be a pb in the SIFT library (?).

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


--------------------------------------------------------------------
Main URL
--------------------------------------------------------------------
http://code.google.com/p/kth-rgbd/


--------------------------------------------------------------------
Dependencies
--------------------------------------------------------------------

* OpenCV
http://opencv.willowgarage.com/
Last tested with 2.3.1

* OpenNI
http://www.openni.org/
Last tested with 1.3.2.1-4

* Point Cloud Libary
http://pointclouds.org/
Last tested with 1.2
(subdependencies -> cminpack, Flann)

* Eigen3
http://eigen.tuxfamily.org/
Last tested with 3.0.2-3

* SIFT Library - Rob Hess
http://blogs.oregonstate.edu/hess/code/sift/

* Boost Library
http://www.boost.org/
Last tested with 1.40.0


--------------------------------------------------------------------
Known bugs
--------------------------------------------------------------------
Frames 2734-2735:	 Extracting SIFT features... OpenCV Error: Null pointer () in cvCvtSeqToArray, file cxcore/cxdatastructs.cpp, line 548
terminate called after throwing an instance of 'cv::Exception'
this may be a pb in the SIFT library (?).

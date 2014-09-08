// kth-rgbd: Visual SLAM from RGB-D data
// Copyright (C) 2011-2013  Virgile Högman
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

Checkout kth-rgbd code from SVN.
svn co https://kth-rgbd.googlecode.com/svn/trunk/kth-rgbd

--------------------------------------------------------------------
Dependencies
--------------------------------------------------------------------
Dependencies have to be installed correctly before building kth-rgbd.
URLs are given for reference, install preferably with package manager.
OpenSIFT and g2o require manual installation. 

* Boost Library
http://www.boost.org/
Requires Boost>=1.46 
Last tested with 1.55

* OpenCV
http://opencv.willowgarage.com/
Last tested with 2.4.5

* OpenNI
https://github.com/OpenNI/OpenNI
Requires OpenNI 1 (not OpenNI 2 SDK) preferably >=1.5
Last tested with 1.5.7

* Point Cloud Libary
http://pointclouds.org/
Requires PCL>=1.5
Last tested with 1.7
(subdependencies -> cminpack, Flann)

* Eigen3
http://eigen.tuxfamily.org/
Last tested with 3.0.2-3

* OpenSIFT Library - Rob Hess
http://robwhess.github.com/opensift/
(subdepencies -> gtk, opencv)

* g2o: A General Framework for Graph Optimization (OpenSLAM)
svn co https://svn.openslam.org/data/svn/g2o

--- Important note for g2o (svn revision 33) ---
A legacy version is found in g2o/tags/before-github-sync.
Build this version instead of the one found in g2o/trunk.
If SuiteSparse is found by g2o (optional package) there will be link errors as this in not supported in this current version of kth-rgbd.
To hide SuiteSparse in g2o, edit g2o/before-github-sync/CMakeLists.txt and comment (#) FIND_PACKAGE(CSPARSE) before running cmake ..
When built done you can check that g2o/before-github-sync/lib contains libg2o_ext_csparse.so.
Use this legacy version of g2o in kth-rgbd (see below). 

--------------------------------------------------------------------
HOWTO build
--------------------------------------------------------------------
Edit kth-rgbd/src/CMakeLists.txt and set the correct paths for:
set(g2o_SOURCE_DIR .../g2o/tags/before-github-sync)
set(OpenSIFT_DIR .../opensift)

Build all using cmake: 
cd kth-rgbd 	#your SVN checkout directory
mkdir build
cd build
cmake ..
make

Potential build problems:
PB: if SuiteSparse package is found by g2o then kth-rgbd will not link
SOL: disable SuiteSparse in g2o.

PB: OpenNI headers like XnCppWrapper.h not found
SOL: be sure to use OpenNI1 (not 2). With OpenNI<1.5 headers are in /usr/include/openni

PB: link problems with GTK and OpenSift
SOL: try by replacing the last command with the last line in src/CMakeLists.txt

--------------------------------------------------------------------
HOWTO run
--------------------------------------------------------------------
Check the configuration file containing the parameters (kth-rgbd.cfg)
No config operation should be required if the scripts are used.
Run those from the script directory.

cd script
./record.sh

[Short description of the scripts]

record: saves a sequence and builds map
sequence: replays a sequence from given frames previously recorded
main: call main program with all the custom arguments (check usage)

archive_results: saves the results into a folder
restore_results: restores the results previously saved
clean: removes current results

--------------------------------------------------------------------
Known bugs
--------------------------------------------------------------------
Main problem is residual drift on vertical axis (Y).

Improve cmake config for g2o and opensift.

--------------------------------------------------------------------
Contact
--------------------------------------------------------------------
Virgile Högman
PhD Student at CVAP, KTH Royal Institute of Technology
virgile@kth.se

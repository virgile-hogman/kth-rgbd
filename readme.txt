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


--------------------------------------------------------------------
Dependencies
--------------------------------------------------------------------

* OpenCV
http://opencv.willowgarage.com/
Last tested with 2.4.5

* OpenNI
http://www.openni.org/
Last tested with 1.3.2.1-4

* Point Cloud Libary
http://pointclouds.org/
Last tested with 1.5
(subdependencies -> cminpack, Flann)

* Eigen3
http://eigen.tuxfamily.org/
Last tested with 3.0.2-3

* OpenSIFT Library - Rob Hess
http://robwhess.github.com/opensift/
(subdepencies -> gtk, opencv)

* Boost Library
http://www.boost.org/
Last tested with 1.46


--------------------------------------------------------------------
HOWTO install
--------------------------------------------------------------------
No installation package provided.
Checkout from SVN and build.

--------------------------------------------------------------------
HOWTO build
--------------------------------------------------------------------
Use cmake and standard build commands.

From installation directory:
mkdir build
cd build
cmake ..
make

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
None so far!
Main problem is residual drift on vertical axis (Y).

--------------------------------------------------------------------
Contact
--------------------------------------------------------------------
Virgile Högman
PhD Student at CVAP, KTH Royal Institute of Technology
virgile@kth.se

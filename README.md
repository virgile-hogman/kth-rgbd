# kth-rgbd
--------------------------------------------------------------------
Visual SLAM from RGB-D data  
Copyright (C) 2011-2015  Virgile Högman  
GNU GPL v3 License.

Mail URL: https://github.com/virgile-hogman/kth-rgbd/  

## Description
--------------------------------------------------------------------
This library developed on Linux produces a 3D map from RGB-D data using Microsoft Kinect.  
It includes a user-friendly program with a GUI to record RGB-D data and produce the 3D map in PCL format. Additional data about the map can be exploited by developers.  
The SLAM approach is based on visual features such as SIFT and SURF.  
To correct the inherent drift errors, loop closures are detected and the map is adjusted with a graph-oriented optimization (using g2o). It is strongly adviced to record loop closures or the optimization cannot be done.  

## License
--------------------------------------------------------------------
Visual SLAM from RGB-D data  
Copyright (C) 2011-2015  Virgile Högman  

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

## Dependencies
--------------------------------------------------------------------
There is no package distributed with kth-rgbd, so dependencies have to be installed correctly before building kth-rgbd.  
OpenSIFT and g2o require manual installation (in local workspace, it is not necessary to install them at system level with root permissions).  
The others dependencies should be installed with standard package managers.  

### g2o
[g2o: A General Framework for Graph Optimization (OpenSLAM)](https://svn.openslam.org/data/svn/g2o)   
Last tested with svn rev 54.  
`svn co https://svn.openslam.org/data/svn/g2o`  
Build the repository version in trunk.


**NOTE! CSPARSE BUILD IN G2O**  
CSparse is an optional package in g2o which provides also a redistribution in EXTERNAL.  
If CSparse is not found by g2o it builds its own version called `libg2o_ext_csparse.so`
kth-rgbd uses this default behavior and looks for CSparse to know which headers and libs to use.  
This default behavior can also be changed with `BUILD_CSPARSE` option (using ccmake) in g2o.
A similar option `G2O_BUILD_CSPARSE` exists in kth-rgbd to force using EXTERNAL version built by g2o.  
In any case kth-rgbd and g2o must be configured accordingly or there will be errors with the linker.

```
cd g2o/trunk
cd build
cmake ..
make
```

Note: the provided configuration kth-rgbd points to the build directory of g2o.
It is not necessary to make install g2o at system level.

### OpenSift
[OpenSIFT Library - Rob Hess](http://robwhess.github.com/opensift/)  
(sub-dependencies: GTK2, OpenCV)  
`git clone https://github.com/robwhess/opensift`

Build the (static) library using MAKEFILE.
```
cd opensift
make
```

Note: the provided configuration kth-rgbd point to the build directory of OpenSIFT.
It is not necessary to make install OpenSIFT at system level.

### Other dependencies
URLs are given for reference, install preferably with standard package managers.
* [OpenCV](http://opencv.willowgarage.com/) Last tested with 2.4.10
* [OpenNI](https://github.com/OpenNI/OpenNI) Last tested with 1.5.7  
Requires OpenNI 1 (not OpenNI 2 SDK) preferably >=1.5  
* [Point Cloud Libary](http://pointclouds.org/) Last tested with 1.7.2  
Requires PCL>=1.7 (sub-dependencies: cminpack, Flann)
* [Boost Library](http://www.boost.org/) Last tested with 1.57   
Requires Boost>=1.46
* [Eigen3](http://eigen.tuxfamily.org/) Last tested with 3.0.2-3  

## Compilation
--------------------------------------------------------------------
Get the source code from the main repository:  
`git clone https://github.com/virgile-hogman/kth-rgbd/`

Edit `kth-rgbd/CMakeLists.txt` and set the correct paths for:
```
set(g2o_SOURCE_DIR .../g2o/trunk)
set(OpenSIFT_DIR .../opensift)
```

The other packages should be found with standard cmake configuration files.

Build kth-rgbd using cmake: 
```sh
cd kth-rgbd 	#your SVN checkout directory
mkdir build
cd build
cmake ..
make
```

No installation step is required. Running scripts call the binary from the proper directory.

**Troubleshooting**
- _if CSparse package is found by g2o then kth-rgbd may not link_  
Configure g2o and kth-rgbd accordingly as described above. Not tested with SuiteSparse yet.
- _OpenNI headers like XnCppWrapper.h not found_  
Be sure to use OpenNI1 (not 2). With OpenNI<1.5 headers are in `/usr/include/openni`
- _link problems with GTK and OpenSift_  
Check instructions in `CMakeLists.txt`

## Execution
--------------------------------------------------------------------
Check the configuration file containing the parameters: `kth-rgbd.cfg`  
No further configuration should be required if the scripts are used.  
Run those from the `script` directory.  
The main purpose is to record scenes with at least one cycle in order to create loop closures.  

```sh
cd script
./record.sh
```

This will open a window displaying the features but the registration is not started.  
Commands can be launched from the GUI window or console.  
- **START**: press any key. Frames are recorded in `DIR_FRAMES`.  
- **STOP**: press ENTER. This will create the map in `DIR_PROD`.  
- **CANCEL**: press ESC. Map is not created but the frames are kept.  

If a loop closure is detected the initial and optimized map are produced.  
If no loop closure is detected only the initial map is produced.  
`DIR_FRAMES` and `DIR_PROD` can be changed in `scripts/var_env.sh`  

**Short description of the scripts** 
- `record`: saves a sequence and builds map  
- `sequence`: replays a sequence from given frames previously recorded  
- `main`: calls main program with all the custom arguments (check usage)  
- `archive_results`: saves the results into a folder  
- `restore_results`: restores the results previously saved  
- `clean`: removes current results  

## Known bugs
--------------------------------------------------------------------
- Main problem is residual drift on vertical axis (Y).
- Enable ICP alignment.
- Redistribution of g2o and OpenSIFT?  

## Contact
--------------------------------------------------------------------
Virgile Högman  
PhD Student at CVAP, KTH Royal Institute of Technology  
virgile@kth.se  

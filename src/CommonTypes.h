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

#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

// -----------------------------------------------------------------------------------------------------
//  common data types
// -----------------------------------------------------------------------------------------------------
typedef	unsigned short	TDepthPixel;	// 16 bits

// -----------------------------------------------------------------------------------------------------
//  constants
// -----------------------------------------------------------------------------------------------------
#define NBPIXELS_WIDTH	640
#define NBPIXELS_HEIGHT	480

#define NBPIXELS_X_HALF 320
#define NBPIXELS_Y_HALF 240

#define MIDDLE_POINT	(640*240 + 320)	// only used for debug test

#define PI 3.14159265
#define PI180 0.017453292519943

// -----------------------------------------------------------------------------------------------------
//  Transformations
// -----------------------------------------------------------------------------------------------------
class Transformation
{
public:
	Eigen::Matrix4f	_matrix;
	double			_error;
	int				_idOrig;
	int				_idDest;
	float			_ratioInliers;
	float			_qualityScore;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// for alignment read http://eigen.tuxfamily.org/dox/UnalignedArrayAssert.html
};

typedef std::vector<Transformation, Eigen::aligned_allocator<Eigen::Vector4f> > TransformationVector;


class Pose
{
public:
	Eigen::Matrix4f	_matrix;
	int				_id;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// for alignment read http://eigen.tuxfamily.org/dox/UnalignedArrayAssert.html
};

typedef std::vector<Pose, Eigen::aligned_allocator<Eigen::Vector4f> > PoseVector;


//vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Vector4f> > g_transformations;


#endif

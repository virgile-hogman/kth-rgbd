#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

// -----------------------------------------------------------------------------------------------------
//  common data types
// -----------------------------------------------------------------------------------------------------
typedef	unsigned short	TDepthPixel;	// 16 bits

#define NBPIXELS_WIDTH	640
#define NBPIXELS_HEIGHT	480

#define NBPIXELS_X_HALF 320
#define NBPIXELS_Y_HALF 240

#define MIDDLE_POINT	(640*240 + 320)	// just for debug

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

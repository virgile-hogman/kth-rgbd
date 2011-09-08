#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <Eigen/Geometry>
#include "FrameData.h"
#include "CommonTypes.h"

class PointCloud
{
public:
	// start a transformation sequence
	static bool getTransformICP(const FrameData &frameData1, const FrameData &frameData2, Eigen::Matrix4f& transformation);

	static void generatePCD(const PoseVector &cameraPoses, const char *filenamePCD);
};

#endif // POINT_CLOUD_H

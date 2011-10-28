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

#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <Eigen/Geometry>
#include "FrameData.h"
#include "CommonTypes.h"

class PointCloud
{
public:
	// start a transformation sequence
	static bool getTransformICP(
			const FrameData &frameData1,
			const FrameData &frameData2,
			const std::vector<Eigen::Vector3f> &source,
			const std::vector<Eigen::Vector3f> &target,
			Eigen::Matrix4f& transformation);

	static void generatePCD(const PoseVector &cameraPoses, const char *filenamePCD);
};

#endif // POINT_CLOUD_H

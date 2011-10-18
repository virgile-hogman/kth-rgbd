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

#ifndef FEATURE_MATCHING_H
#define FEATURE_MATCHING_H

#include "CommonTypes.h"
#include "FrameData.h"

bool computeTransformation(
		int frameID1,
		int frameID2,
		FrameData &frameData1,
		FrameData &frameData2,
		Transformation &resultingTransfo,
		bool forLoopClosure=false);

bool checkLoopClosure(
		int frameID1,
		int frameID2,
		FrameData &frameData1,
		FrameData &frameData2,
		Transformation &resultingTransfo);

#endif // FEATURE_MATCHING_H

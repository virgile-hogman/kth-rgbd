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

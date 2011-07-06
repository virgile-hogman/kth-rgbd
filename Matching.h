#ifndef FEATURE_MATCHING_H
#define FEATURE_MATCHING_H

#include "CommonTypes.h"
#include "FrameData.h"

bool matchFrames(
		int frameID1,
		int frameID2,
		FrameData &frameData1,
		FrameData &frameData2,
		bool methodRandom,
		Transformation &resultingTransfo);

#endif // FEATURE_MATCHING_H
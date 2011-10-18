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

#ifndef CONFIG_H
#define CONFIG_H

#include <string>

class Config
{
public:
	static void LoadConfig(std::string filename);

	// data and files
	static std::string		_DataDirectory;
	static std::string		_GenDirectory;
	static std::string		_ResultDirectory;
	static std::string		_PathKinectXmlFile;
	static int				_DataInRatioFrame;
	static bool				_SaveImageInitialPairs;

	// feature
	static int				_FeatureType;
	static int				_FeatureDepthMin;	// min depth in mm
	static int				_FeatureDepthMax;	// max depth in mm
	static bool				_FeatureDisplay;	// display windows

	// matching
	static float			_MatchingDistanceRatioNN;
	static float			_MatchingMaxDistanceKeypoint;
	static bool				_MatchingAllowInvalid;
	static int				_MatchingNbIterations;
	static int				_MatchingMinNbInlier;
	static float			_MatchingMinRatioInlier;
	static float			_MatchingMaxDistanceInlier;
	static bool				_MatchingRunICP;

	// mapping
	static float			_MapNodeDistance;
	static float			_MapNodeAngle;

	// triggering loop closure
	static float			_LoopClosureDistance;
	static float			_LoopClosureAngle;
	static int				_LoopClosureWindowSize;

	static bool				_PcdGenerateInitial;
	static bool				_PcdGenerateOptimized;
	static int				_PcdRatioKeepSubsample;
	static int				_PcdMaxNbPoints;
	static int				_PcdRatioFrame;
};

#endif //CONFIG_H


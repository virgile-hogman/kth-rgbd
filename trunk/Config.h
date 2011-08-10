#ifndef CONFIG_H
#define CONFIG_H

#include <string>

class Config
{
public:
	static void LoadConfig(std::string filename);

	static std::string		_DataDirectory;
	static std::string		_GenDirectory;
	static std::string		_ResultDirectory;

	// feature
	static int				_FeatureDepthMin;	// min depth in mm
	static int				_FeatureDepthMax;	// max depth in mm

	// matching
	static int				_MatchingRatioFrame;
	static bool				_MatchingAllowInvalid;
	static bool				_MatchingSaveImageInitialPairs;
	static int				_MatchingNbIterations;
	static int				_MatchingMinNbInlier;
	static float			_MatchingMinRatioInlier;
	static float			_MatchingMaxDistanceInlier;

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


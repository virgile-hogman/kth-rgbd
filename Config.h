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

	// triggering loop closure
	static float			_LoopClosureDistance;
	static float			_LoopClosureAngle;
	static int				_LoopClosureWindowSize;

	static bool				_GenerateInitialPCD;
	static bool				_GenerateOptimizedPCD;
	static int				_RatioKeepSubsamplePCD;
	static int				_MaxNbPointsPCD;
	static int				_RatioFramePCD;
};

#endif //CONFIG_H


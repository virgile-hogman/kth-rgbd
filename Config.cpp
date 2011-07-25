#include "Config.h"
#include "ConfigFile.h"

std::string Config::_DataDirectory = "data_in";
std::string Config::_GenDirectory = "data_gen";
std::string Config::_ResultDirectory = "data_out";

float		Config::_LoopClosureDistance = 6.0;
float		Config::_LoopClosureAngle = 300.0;
int			Config::_LoopClosureWindowSize = 5;

bool		Config::_GenerateInitialPCD = true;
bool		Config::_GenerateOptimizedPCD = true;
int			Config::_RatioKeepSubsamplePCD = 80;
int			Config::_MaxNbPointsPCD = 2E6;
int			Config::_RatioFramePCD = 1;

void Config::LoadConfig(std::string filename)
{
	ConfigFile config(filename);

	Config::_DataDirectory = config.read<string>("data_in", "data_in");
	Config::_GenDirectory = config.read<string>("data_gen", "data_gen");
	Config::_ResultDirectory = config.read<string>("data_out", "data_out");

	Config::_LoopClosureDistance = config.read<float>("LoopClosureDistance", 6.0);
	Config::_LoopClosureAngle = config.read<float>("LoopClosureAngle", 300.0);
	Config::_LoopClosureWindowSize = config.read<int>("LoopClosureWindowSize", 5);

	Config::_GenerateInitialPCD = config.read<bool>("GenerateInitialPCD", true);
	Config::_GenerateOptimizedPCD = config.read<bool>("GenerateOptimizedPCD", true);
	Config::_RatioKeepSubsamplePCD = config.read<int>("RatioKeepSubsamplePCD", 80);
	Config::_MaxNbPointsPCD = config.read<int>("MaxNbPointsPCD", 2E6);
	Config::_RatioFramePCD = config.read<int>("RatioFramePCD", 1);
}

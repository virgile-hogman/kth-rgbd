#include "Config.h"

std::string Config::_DataDirectory = "data_in";
std::string Config::_GenDirectory = "data_gen";
std::string Config::_ResultDirectory = "data_out";

float		Config::_LoopClosureDistance = 8.0;
float		Config::_LoopClosureAngle = 300.0;
int			Config::_LoopClosureWindowSize = 5;

bool		Config::_GenerateInitialPCD = false;
bool		Config::_GenerateOptimizedPCD = true;

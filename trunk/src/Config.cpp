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

#include "Config.h"
#include "ConfigFile.h"

// presets
#define		FOCAL_LENGTH_KINECT		575.815735

std::string Config::_PathFrameSequence = "rgbd_frames";
std::string Config::_PathDataProd = "rgbd_prod";
std::string	Config::_PathKinectXmlFile = "KinectConfig.xml";
int			Config::_DataInRatioFrame = 1;
bool		Config::_SaveImageInitialPairs = false;

int			Config::_FeatureType = 0;
int			Config::_FeatureDepthMin = 0;
int			Config::_FeatureDepthMax = 6000;
bool		Config::_FeatureDisplay = false;

float		Config::_MatchingDistanceRatioNN = 0.5;
float		Config::_MatchingMaxDistanceKeypoint = 3.0;
bool		Config::_MatchingAllowInvalid = false;
int			Config::_MatchingNbIterations = 20;
int			Config::_MatchingMinNbInlier = 10;
float		Config::_MatchingMinRatioInlier = 0.3;
float		Config::_MatchingMaxDistanceInlier = 0.05;
bool		Config::_MatchingRunICP = false;

float		Config::_MapInitialAngle[3] = {0,0,0};
float		Config::_MapInitialCoord[3] = {0,0,0};
float		Config::_MapNodeDistance = 0.1;
float		Config::_MapNodeAngle = 5.0;

int			Config::_LoopClosureWindowSize = 5;
int			Config::_LoopClosureExcludeLast = 5;

bool		Config::_PcdGenerateInitial = true;
bool		Config::_PcdGenerateOptimized = true;
int			Config::_PcdRatioKeepSubsample = 80;
int			Config::_PcdMaxNbPoints = 2E6;
int			Config::_PcdRatioFrame = 1;

float		Config::_FocalLength = FOCAL_LENGTH_KINECT;

void Config::LoadConfig(std::string filename)
{
	ConfigFile config(filename);

	Config::_PathFrameSequence = config.read<string>("PathFrameSequence", "rgbd_frames");
	Config::_PathDataProd = config.read<string>("PathDataProd", "rgbd_prod");
	Config::_PathKinectXmlFile = config.read<string>("PathKinectXmlFile", "KinectConfig.xml");

	Config::_DataInRatioFrame = config.read<int>("DataInRatioFrame", 0);
	Config::_SaveImageInitialPairs = config.read<bool>("SaveImageInitialPairs", false);

	Config::_FeatureType = config.read<int>("FeatureType", 0);
	Config::_FeatureDepthMin = config.read<int>("FeatureDepthMin", 0);
	Config::_FeatureDepthMax = config.read<int>("FeatureDepthMax", 6000);
	Config::_FeatureDisplay = config.read<bool>("FeatureDisplay", false);

	Config::_MatchingDistanceRatioNN = config.read<float>("MatchingDistanceRatioNN", 0.5);
	Config::_MatchingMaxDistanceKeypoint = config.read<float>("MatchingMaxDistanceKeypoint", 3.0);
	Config::_MatchingAllowInvalid = config.read<bool>("MatchingAllowInvalid", false);
	Config::_MatchingNbIterations = config.read<int>("MatchingNbIterations", 20);
	Config::_MatchingMinNbInlier = config.read<int>("MatchingMinNbInlier", 10);
	Config::_MatchingMinRatioInlier = config.read<float>("MatchingMinRatioInlier", 0.3);
	Config::_MatchingMaxDistanceInlier = config.read<float>("MatchingMaxDistanceInlier", 0.05);
	Config::_MatchingRunICP = config.read<bool>("MatchingRunICP",false);

	Config::_MapInitialAngle[0] = config.read<float>("MapInitialAngleX", 0);
	Config::_MapInitialAngle[1] = config.read<float>("MapInitialAngleY", 0);
	Config::_MapInitialAngle[2] = config.read<float>("MapInitialAngleZ", 0);
	Config::_MapInitialCoord[0] = config.read<float>("MapInitialCoordX", 0);
	Config::_MapInitialCoord[1] = config.read<float>("MapInitialCoordY", 0);
	Config::_MapInitialCoord[2] = config.read<float>("MapInitialCoordZ", 0);
	Config::_MapNodeDistance = config.read<float>("MapNodeDistance", 0.1);
	Config::_MapNodeAngle = config.read<float>("MapNodeAngle", 5.0);

	Config::_LoopClosureWindowSize = config.read<int>("LoopClosureWindowSize", 5);
	Config::_LoopClosureExcludeLast = config.read<int>("LoopClosureExcluseLast", 5);

	Config::_PcdGenerateInitial = config.read<bool>("PcdGenerateInitial", true);
	Config::_PcdGenerateOptimized = config.read<bool>("PcdGenerateOptimized", true);
	Config::_PcdRatioKeepSubsample = config.read<int>("PcdRatioKeepSubsample", 80);
	Config::_PcdMaxNbPoints = config.read<int>("PcdMaxNbPoints", 2E6);
	Config::_PcdRatioFrame = config.read<int>("PcdRatioFrame", 1);

	Config::_FocalLength = config.read<float>("FocalLength", FOCAL_LENGTH_KINECT);
}

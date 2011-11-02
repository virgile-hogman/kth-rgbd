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

#include <boost/filesystem.hpp>

#include "Config.h"
#include "FrameData.h"
#include "Sequence.h"
#include "Matching.h"
#include "TimeTracker.h"

// standard
#include <iostream>
#include <stdio.h>

using namespace std;

// name of the configuration file where all the parameters are set
#define CONFIG_FILENAME	"rgbd_params.cfg"

void printUsage(const char *name)
{
	printf("\nUsage: %s <options>\n", name);
	printf("\nStandard options:\n");
	printf(" -r:\t record data from camera, run sequence and build map\n");
	printf(" -s <idFrom> <idTo> [ratio_frame]:\t run a sequence of frames (RGB-D files), compute transformations and build map\n");
	printf(" -m <idFrom> <idTo>:\t build map from existing transformations (transfo.dat), generate poses, graph and PCD\n");
	printf(" -p [ratio_pcd]:\t regenerate PCD file from existing positions (poses_optimized.g2o)\n");
	printf("\nMore options:\n");
	printf(" -f <idFrom> <idTo>:\t compute features for each frame in given range\n");
	printf(" -t <id1> <id2>:\t match and compute transformation for couple of frames\n");
	printf("\n");
}

// -----------------------------------------------------------------------------------------------------
//  Main program
// -----------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
	vector<int> sequenceFramesID;
	Sequence sequence;
	TimeTracker tm;

    if (argc<2)
	{
		printUsage(argv[0]);
		return -1;
	}
	
	// load configuration
	Config::LoadConfig(CONFIG_FILENAME);
    FrameData::_DataPath = Config::_PathFrameSequence;

	srand(time(NULL));

    // ---------------------------------------------------------------------------------------------------
    //  regenerate PCD files from archive
    // ---------------------------------------------------------------------------------------------------
    if (strcmp(argv[1], "-p") == 0)
    {
        printf("-Regenerate PCD files from graph archive-\n");

    	if ( ! boost::filesystem::exists( Config::_PathFrameSequence ) )
    		return -1;
    	if ( ! boost::filesystem::exists( Config::_PathDataProd ) )
    		return -1;

    	if (argc>2) {
    		int param = atoi(argv[2]);
    		if (param != 0) {
    			Config::_PcdRatioFrame = param;	// override PCD ratio
    			printf("Override ratio frame: ratio=%d\n", Config::_PcdRatioFrame);
    		}
    	}

    	sequence.regeneratePCD();
    }
    // ---------------------------------------------------------------------------------------------------
    //  map reconstruction from transformations archive
    // ---------------------------------------------------------------------------------------------------
    else if (strcmp(argv[1], "-m") == 0)
    {
        printf("-Build map from transformations archive, with loop closure-\n");

    	if ( ! boost::filesystem::exists( Config::_PathFrameSequence ) )
    		return -1;
    	if ( ! boost::filesystem::exists( Config::_PathDataProd ) )
    		return -1;

    	int min=-1, max=-1;
    	if (argc>3)
    		max = atoi(argv[3]);
    	if (argc>2)
    		min = atoi(argv[2]);

    	sequence.restoreTransformations(min, max);
    	sequence.buildMap();
    }
    // ---------------------------------------------------------------------------------------------------
    //  load sequence from RGB+D input datafiles
    // ---------------------------------------------------------------------------------------------------
    else if (strcmp(argv[1], "-s") == 0)
    {
        printf("-Replay a sequence of frames-\n");
    	if ( ! boost::filesystem::exists( Config::_PathFrameSequence ) )
    		return -1;

    	int min=-1, max=-1;
    	if (argc>3)
    		max = atoi(argv[3]);
    	if (argc>2)
    		min = atoi(argv[2]);
    	
    	if (argc>4) {
    		int param = atoi(argv[4]);
			if (param != 0) {
				Config::_DataInRatioFrame = param;	// override ratio frame
				printf("Override ratio frame: ratio=%d\n", Config::_DataInRatioFrame);
			}
    	}

        boost::filesystem::create_directories(Config::_PathDataProd);
		
        tm.start();
     	sequence.reloadFrames(min, max);
    	tm.stop();
		printf("Matching duration time: %d(ms)\n", tm.duration());
    	// build map
    	sequence.buildMap();
	}
    // ---------------------------------------------------------------------------------------------------
    //  acquire data from camera and build map
    // ---------------------------------------------------------------------------------------------------
    else if (strcmp(argv[1], "-r") == 0)
    {
        printf("-Record sequence from camera-\n");
        
        boost::filesystem::create_directories(Config::_PathFrameSequence);
        boost::filesystem::create_directories(Config::_PathDataProd);

        // folder receiving frame sequence must be empty
        if (! boost::filesystem::is_empty(Config::_PathFrameSequence)) {
        	printf("Directory [%s] must be empty!\n", Config::_PathFrameSequence.c_str());
        	return -1;
        }

        // record and build map
        sequence.recordFrames(true);
    }
    // ---------------------------------------------------------------------------------------------------
    //  compute feature for a single frame
    // ---------------------------------------------------------------------------------------------------
    else if (strcmp(argv[1], "-f") == 0)
    {
    	TimeTracker tm;
    	FrameData frameData;
    	int frameID1, frameID2;

        boost::filesystem::create_directories(Config::_PathDataProd);

    	std::ofstream fileStats;
    	char buf[256];
    	sprintf(buf, "%s/stats_features.log", Config::_PathDataProd.c_str());
    	fileStats.open(buf);

    	if (argc>2)	{
    		frameID1 = atoi(argv[2]);
        	if (argc>3)
        		frameID2 = atoi(argv[3]);
        	else
        		frameID2 = frameID1;

        	bool saveImage = true;
        	if (argc>4)	{
        		// override export (read boolean)
        		saveImage = (atoi(argv[4])!=0);
        	}
        	if (argc>5)	{
        		// override type
        		Config::_FeatureType = atoi(argv[5]);
        	}

        	for (int id=frameID1; id<=frameID2; id++) {

				if (frameData.loadImageRGBD(id)) {
					// save image with features
					tm.start();
					frameData.computeFeatures();
					tm.stop();
					printf("Id=%d %d features.\t(%dms)\n", id, frameData.getNbFeatures(), tm.duration());
					fileStats << frameData.getFrameID() << "\t";
					fileStats << frameData.getNbFeatures() << "\t";
					fileStats << tm.duration() << "\n";
					if (saveImage) {
						frameData.drawFeatures();
						frameData.saveImageRGB(Config::_PathDataProd.c_str());
					}
				}
				else
					printf("Failed to load data ID=%d\n", id);
				frameData.releaseData();
        	}
    	}
    }
    // ---------------------------------------------------------------------------------------------------
    //  match frames and compute transformation 2 by 2 for given range of data
    // ---------------------------------------------------------------------------------------------------
    else if (strcmp(argv[1], "-t") == 0)
    {
    	TimeTracker tm;
    	FrameData frameData1, frameData2;
    	int frameID1, frameID2;
    	Transformation transform;

        boost::filesystem::create_directories(Config::_PathDataProd);

        // force matching export
        Config::_SaveImageInitialPairs = true;
    	if (argc>4)	{
    		// override export (read boolean)
            Config::_SaveImageInitialPairs = (atoi(argv[4])!=0);
    	}

    	if (argc>3)	{
    		frameID1 = atoi(argv[2]);
    		frameID2 = atoi(argv[3]);

			tm.start();
    		// match frame to frame (current with previous)
    		bool match = computeTransformation(
    				frameID1,
    				frameID2,
    				frameData1,
    				frameData2,
    				transform);
			tm.stop();
			printf("Transformation is %s valid.\t(%dms)\n", match?"":"not", tm.duration());
    	}
    }
    else {
		printUsage(argv[0]);
    	return -1;
    }
    
    return 0;
}

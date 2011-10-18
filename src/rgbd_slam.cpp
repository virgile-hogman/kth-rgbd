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
#include "CameraDevice.h"
#include "FrameData.h"
#include "Map.h"
#include "Matching.h"
#include "TimeTracker.h"

// standard
#include <iostream>
#include <stdio.h>

#include <vector>
#include <list>

using namespace std;

// -----------------------------------------------------------------------------------------------------
//  loadSequence
// -----------------------------------------------------------------------------------------------------
void loadSequence(const char *dataDirectory, int min, int max, vector<int> &sequenceFramesID)
{
	int frameID;
	list<int> listFramesID;
	
	sequenceFramesID.clear();
	
	boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
	for ( boost::filesystem::directory_iterator itr( dataDirectory );
		itr != end_itr;
		++itr )
	{
		//	printf("%s\t%s\n", itr->leaf().c_str(), itr->path().string().c_str());
		if (boost::filesystem::extension(*itr)==".bmp" &&
			sscanf(itr->leaf().c_str(), "frame_%d", &frameID)==1 && frameID>=min && (frameID<=max || max<0))
		{
			// add frame
			//printf("Add frame file #%i:\t%s\n", frameID, itr->path().string().c_str());
			listFramesID.push_back(frameID);
		}
	}
	// sort and keep only 1 element (remove duplicates because of rgb+depth)
	listFramesID.sort();
	listFramesID.unique();
	
	// build the sequence
	while (!listFramesID.empty())
	{
		sequenceFramesID.push_back(listFramesID.front());
		listFramesID.pop_front();
	}

	cout << "Sequence of " << sequenceFramesID.size() << " frames available.\n";
	cout << "Sequence of " << sequenceFramesID.size()/Config::_DataInRatioFrame << " frames with ";
	cout << "Frame ratio:" << Config::_DataInRatioFrame << "\n";
}

void recordSequence(Map &map)
{
	CameraDevice cameraKinect;
	TimeTracker tm;

    if (cameraKinect.connect())
    {
    	int frameID = 0;
    	Transformation transform;

        boost::filesystem::create_directories(Config::_ResultDirectory);

        tm.start();

    	// generate 1st frame
        if (cameraKinect.generateFrame(frameID))
        {
			frameID++;

			map.initSequence();

        	// generate new frame
			while (cameraKinect.generateFrame(frameID))
			{
				// associate with previous
				if (! map.addFrames(frameID-1, frameID, transform))
				{
					printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
					printf("!!! LOW QUALITY !!! LOST SYNCHRO !!!\n");
					printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
					printf("Last Frame (%d) rejected\n", frameID);
				}
				else
					frameID++;
			}

			tm.stop();
			printf("Acquisition and matching duration time: %d(ms)\n", tm.duration());

	    	// build map
			if (!cameraKinect.aborted())
				map.build();
			else
				printf("Aborted.\n");
        }

    	cameraKinect.disconnect();
    }
}

void printUsage(const char *name)
{
	printf("Usage: %s <options>\n", name);
	printf(" -record: acquire data from camera, match and build map\n");
	printf(" -match <from> <to>: build transformations and map");
	printf(" -map <from> <to>: build map (with loop closure) from existing transformation (transfo.dat)\n");
	printf(" -pcd: generate PCD file from existing positions (poses.dat)\n");
	printf(" -feature <from> <to>: compute features for image data in given range\n");
	printf(" -transfo <from> <to>: compute transformations for image data in given range\n");
}

// -----------------------------------------------------------------------------------------------------
//  Main program
// -----------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
	vector<int> sequenceFramesID;
	Map map;
	TimeTracker tm;
    
	if (argc<1)
	{
		printUsage(argv[0]);
		return -1;
	}
	
	// load configuration
	Config::LoadConfig("kth-rgbd.cfg");
    FrameData::_DataPath = Config::_DataDirectory;

	srand(time(NULL));

    // ---------------------------------------------------------------------------------------------------
    //  regenerate PCD files from archive
    // ---------------------------------------------------------------------------------------------------
    if (strcmp(argv[1], "-pcd") == 0)
    {
        printf("-Regenerate PCD files from graph archive-\n");

    	if ( ! boost::filesystem::exists( Config::_DataDirectory ) )
    		return -1;
    	if ( ! boost::filesystem::exists( Config::_ResultDirectory ) )
    		return -1;

    	map.regeneratePCD();
    }
    // ---------------------------------------------------------------------------------------------------
    //  map reconstruction from transformations archive
    // ---------------------------------------------------------------------------------------------------
    else if (strcmp(argv[1], "-map") == 0)
    {
        printf("-Reconstruct map from transformations archive, with loop closure-\n");

    	if ( ! boost::filesystem::exists( Config::_DataDirectory ) )
    		return -1;
    	if ( ! boost::filesystem::exists( Config::_ResultDirectory ) )
    		return -1;

    	int min=-1, max=-1;
    	if (argc>3)
    		max = atoi(argv[3]);
    	if (argc>2)
    		min = atoi(argv[2]);

    	map.restoreSequence(min, max);
    	map.build();
    }
    // ---------------------------------------------------------------------------------------------------
    //  load sequence from RGB+D input datafiles
    // ---------------------------------------------------------------------------------------------------
    else if (strcmp(argv[1], "-match") == 0)
    {
        printf("-Match and map a sequence of frames-\n");
    	if ( ! boost::filesystem::exists( Config::_DataDirectory ) )
    		return -1;

    	int min=-1, max=-1;
    	if (argc>3)
    		max = atoi(argv[3]);
    	if (argc>2)
    		min = atoi(argv[2]);
    	
     	loadSequence(Config::_DataDirectory.c_str(), min, max, sequenceFramesID);
    	
        boost::filesystem::create_directories(Config::_ResultDirectory);       
		
    	// build map 
        map.initSequence();
        tm.start();
    	map.addSequence(sequenceFramesID);
    	tm.stop();
		printf("Matching duration time: %d(ms)\n", tm.duration());
    	map.build();
	}
    // ---------------------------------------------------------------------------------------------------
    //  acquire data from camera and build map
    // ---------------------------------------------------------------------------------------------------
    else if (strcmp(argv[1], "-record") == 0)
    {
        printf("-Record sequence from camera-\n");
        
        // use the same directory to generate and reload the data
        Config::_DataDirectory = Config::_GenDirectory;
        FrameData::_DataPath = Config::_GenDirectory;
    		
        boost::filesystem::create_directories(Config::_DataDirectory);
        boost::filesystem::create_directories(Config::_ResultDirectory);

        recordSequence(map);
    }
    // ---------------------------------------------------------------------------------------------------
    //  compute feature for a single frame
    // ---------------------------------------------------------------------------------------------------
    else if (strcmp(argv[1], "-feature") == 0)
    {
    	TimeTracker tm;
    	FrameData frameData;
    	int frameID1, frameID2;

        boost::filesystem::create_directories(Config::_ResultDirectory);

    	if (argc>2)	{
    		frameID1 = atoi(argv[2]);
        	if (argc>3)
        		frameID2 = atoi(argv[3]);
        	else
        		frameID2 = frameID1;

        	for (int id=frameID1; id<=frameID2; id++) {

				if (frameData.loadImage(id) && frameData.loadDepthData()) {
					// save image with features
					tm.start();
					frameData.computeFeatures();
					tm.stop();
					frameData.drawFeatures();
					printf("%d features.\t(%dms)\n", frameData.getNbFeatures(), tm.duration());
					frameData.saveImage();
				}
				frameData.releaseData();
        	}
    	}
    }
    // ---------------------------------------------------------------------------------------------------
    //  compute transfo for a single pair of frames
    // ---------------------------------------------------------------------------------------------------
    else if (strcmp(argv[1], "-transfo") == 0)
    {
    	TimeTracker tm;
    	FrameData frameData1, frameData2;
    	int frameID1, frameID2;
    	Transformation transform;

        boost::filesystem::create_directories(Config::_ResultDirectory);
        // force matching export
        Config::_SaveImageInitialPairs = true;

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

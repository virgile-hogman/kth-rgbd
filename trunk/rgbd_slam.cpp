#include <boost/filesystem.hpp>

#include "Config.h"
#include "CameraDevice.h"
#include "FrameData.h"
#include "Map.h"

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
	cout << "Sequence of " << sequenceFramesID.size()/Config::_MatchingRatioFrame << " frames with ";
	cout << "Frame ratio:" << Config::_MatchingRatioFrame << "\n";
}

void recordSequence(Map &map)
{
	CameraDevice cameraKinect;

    if (cameraKinect.connect())
    {
    	int frameID = 0;
    	Transformation transform;

        boost::filesystem::create_directories(Config::_ResultDirectory);

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
					printf("Last Frame (%d) rejected, it will be overwritten.\n", frameID);
				}
				else
					frameID++;
			}

	    	// build map
	    	map.build();
        }

    	cameraKinect.disconnect();
    }
}

// -----------------------------------------------------------------------------------------------------
//  Main program
// -----------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
	vector<int> sequenceFramesID;
	Map map;
    
	if (argc<1)
	{
		printf("Usage: %s -record | -match <from> <to> | -map <from> <to> | -pcd", argv[0]);
		return -1;
	}
	
	// load configuration
	Config::LoadConfig("kth-rgbd.cfg");
    FrameData::_DataPath = Config::_DataDirectory;

    // ---------------------------------------------------------------------------------------------------
    //  regenerate PCD files from archive
    // ---------------------------------------------------------------------------------------------------
    if (strcmp(argv[1], "-pcd") == 0)
    {
        printf("Regenerate PCD files from graph archive\n");

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
        printf("Reconstruct map from transformations archive, with loop closure\n");

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
        printf("Match and map a sequence of frames\n");
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
    	map.addSequence(sequenceFramesID);
    	map.build();
	}
    // ---------------------------------------------------------------------------------------------------
    //  acquire data from camera and build map
    // ---------------------------------------------------------------------------------------------------
    else if (strcmp(argv[1], "-record") == 0)
    {
        printf("Record sequence from camera\n");
        
        // use the same directory to generate and reload the data
        Config::_DataDirectory = Config::_GenDirectory;
        FrameData::_DataPath = Config::_GenDirectory;
    		
        boost::filesystem::create_directories(Config::_DataDirectory);
        boost::filesystem::create_directories(Config::_ResultDirectory);

        recordSequence(map);
    }
    else {
    	printf("Select a valid option");
    	return -1;
    }
    
    return 0;
}

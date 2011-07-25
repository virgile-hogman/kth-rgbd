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
void loadSequence(const char *dataDirectory, int skipCount, int min, int max, vector<int> &sequenceFramesID)
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
	int counter = 0;
	cout << "Sequence of frames: (";
	while (!listFramesID.empty())
	{
		if (skipCount<=0 || counter==0 || counter>=skipCount)
		{
			cout << " " << listFramesID.front();
			sequenceFramesID.push_back(listFramesID.front());
			if (counter>0)
				counter = 0;
		}
		listFramesID.pop_front();
		counter++;
	}
	cout << ") for " << sequenceFramesID.size() << " frames." << std::endl;
}

// -----------------------------------------------------------------------------------------------------
//  Main program
// -----------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
	vector<int> sequenceFramesID;
	int nbFrames=2;	// by default
	Map map;
    
	if (argc<1)
	{
		printf("Usage: %s --record <nbFrames> | --replay <from> <to> <skip> | --reconstruct", argv[0]);
		return -1;
	}
	
	// load configuration
	Config::LoadConfig("kth-rgbd.cfg");
    FrameData::_DataPath = Config::_DataDirectory;

    // ---------------------------------------------------------------------------------------------------
    //  load sequence from RGB+D input datafiles
    // ---------------------------------------------------------------------------------------------------
    if (strcmp(argv[1], "--replay") == 0)
    {
        printf("Replay sequence\n");
    	if ( ! boost::filesystem::exists( Config::_DataDirectory ) )
    		return -1;

    	int skip=0, min=-1, max=-1;
    	if (argc>4)
    		skip = atoi(argv[4]);
    	if (argc>3)
    		max = atoi(argv[3]);
    	if (argc>2)
    		min = atoi(argv[2]);
    	
    	loadSequence(Config::_DataDirectory.c_str(), skip, min, max, sequenceFramesID);
    	
        boost::filesystem::create_directories(Config::_ResultDirectory);       
		
    	// build map 
    	map.buildFromSequence(sequenceFramesID);
	}
    // ---------------------------------------------------------------------------------------------------
    //  map reconstruction from transformations archive
    // ---------------------------------------------------------------------------------------------------
    else if (strcmp(argv[1], "--reconstruct") == 0)
    {
        printf("Reconstruct map from archive\n");
        
    	// build map from existing transformation data
    	if ( ! boost::filesystem::exists( Config::_DataDirectory ) )
    		return -1;
    	if ( ! boost::filesystem::exists( Config::_ResultDirectory ) )
    		return -1;
    	
    	map.buildFromArchive();
    }
    // ---------------------------------------------------------------------------------------------------
    //  regenerate PCD files from archive
    // ---------------------------------------------------------------------------------------------------
    else if (strcmp(argv[1], "--regenerate") == 0)
    {
        printf("Regenerate PCD files from archive\n");

    	// build map from existing transformation data
    	if ( ! boost::filesystem::exists( Config::_DataDirectory ) )
    		return -1;
    	if ( ! boost::filesystem::exists( Config::_ResultDirectory ) )
    		return -1;

    	map.regeneratePCD();
    }
    // ---------------------------------------------------------------------------------------------------
    //  acquire data from camera and build map
    // ---------------------------------------------------------------------------------------------------
    else if (strcmp(argv[1], "--record") == 0)
    {
    	CameraDevice cameraKinect;
        printf("Record sequence from camera\n");
        
        // use the same directory to generate and reload the data
        Config::_DataDirectory = Config::_GenDirectory;
        FrameData::_DataPath = Config::_GenDirectory;
    		
    	if (nbFrames<2)
    	{
    		printf("At least 2 frames are required!\n");    		
    		return -1;
    	}
        boost::filesystem::create_directories(Config::_DataDirectory);       

        if (cameraKinect.connect())
        {
            boost::filesystem::create_directories(Config::_ResultDirectory);
            
        	// generate n frames and get the sequence
            cameraKinect.generateFrames(nbFrames, sequenceFramesID);
        
        	// build map 
        	map.buildFromSequence(sequenceFramesID);
        	
        	cameraKinect.disconnect();
        }
    }
    else {
    	printf("Select a valid option");
    	return -1;
    }
    
    return 0;
}

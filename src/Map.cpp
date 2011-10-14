#include <Eigen/Geometry>

#include "highgui.h"

#include "Config.h"
#include "CommonTypes.h"
#include "Matching.h"
#include "FrameData.h"
#include "CameraDevice.h"
#include "Map.h"
#include "PointCloud.h"

#include <iostream>

using namespace std;

double getDistanceTransform(const Eigen::Matrix4f &transformMat)
{
	Eigen::Vector3d translation(transformMat(0,3), transformMat(1,3), transformMat(2,3));
	return translation.norm();
}

void getAngleTransform(const Eigen::Matrix4f &transformMat, double angleEuler[3])
{
    // roll
	angleEuler[0]= atan2(transformMat(2,1),transformMat(2,2));
    //pitch
	angleEuler[1]= acos((transformMat(0,0)+transformMat(1,1)+transformMat(2,2) - 1) /2);
    // yaw
	angleEuler[2]= atan2(transformMat(1,0),transformMat(0,0));
}

bool isKeyTransform(const Eigen::Matrix4f &transformMat)
{
	double maxAngle = Config::_MapNodeAngle * PI180;	// convert to radians
	double angle[3];
	getAngleTransform(transformMat, angle);
	return (fabs(angle[0]) > maxAngle ||
			fabs(angle[1]) > maxAngle ||
			fabs(angle[2]) > maxAngle ||
			getDistanceTransform(transformMat) > Config::_MapNodeDistance);
}

// -----------------------------------------------------------------------------------------------------
//  loadPoses (NOT TESTED!!!)
// -----------------------------------------------------------------------------------------------------
void loadPoses(PoseVector &poses, const char *filename)
{
	char buf[256];
	sprintf(buf, "%s/%s", Config::_ResultDirectory.c_str(), filename);
	std::ifstream filePoses(buf);
	Pose pose;

	poses.clear();

	if (filePoses.is_open())
	{
		while (! filePoses.eof())
		{
			filePoses.ignore(256, '#');	// to reach id
			filePoses >> pose._id;
			for (int irow=0; irow<4; irow++)
				for (int icol=0; icol<4; icol++)
					filePoses >> pose._matrix(irow,icol);

			//std::cout << transfo._matrix << std::endl;
			poses.push_back(pose);

			filePoses.ignore(256, '\n');	// to reach end of line
			filePoses.peek();				// to update the eof flag (if no empty line)
		}
	}
}

// -----------------------------------------------------------------------------------------------------
//  savePoses
// -----------------------------------------------------------------------------------------------------
void savePoses(const PoseVector &poses, const char *filename)
{
	char buf[256];
	sprintf(buf, "%s/%s", Config::_ResultDirectory.c_str(), filename);
	std::ofstream filePoses(buf);

	for (int i=0; i<poses.size(); i++)
	{
		filePoses << "Pose [" << i << "] - Frame #" << poses[i]._id << "\n";
		filePoses << poses[i]._matrix << "\n----------------------------------------\n";
	}
}

// -----------------------------------------------------------------------------------------------------
//  regeneratePCD
// -----------------------------------------------------------------------------------------------------
void Map::regeneratePCD()
{
	PoseVector	cameraPoses;

	// initia graph
    _graphOptimizer.initialize();

    _graphOptimizer.load("graph_initial.g2o");

	// extract the updated camera positions from the optimized graph
    _graphOptimizer.extractAllPoses(cameraPoses);

	// build initial point cloud
	if (Config::_PcdGenerateInitial)
		PointCloud::generatePCD(cameraPoses, "cloud_initial");

	//  optimized graph
	_graphOptimizer.load("graph_optimized.g2o");

	// extract the updated camera positions from the optimized graph
	_graphOptimizer.extractAllPoses(cameraPoses);

	// generate optimized point cloud
	if (Config::_PcdGenerateOptimized)
		PointCloud::generatePCD(cameraPoses, "cloud_optimized");
}

// -----------------------------------------------------------------------------------------------------
//  selectCandidateLcRandom
// -----------------------------------------------------------------------------------------------------
int selectCandidateLcRandom(vector<int> &candidates)
{
	if (candidates.size()<1)
		return -1;
	// pickup a candidate randomly
	int indexRandom = rand() % candidates.size();
	int indexLC = candidates[indexRandom];
	// remove this item from the list
	candidates.erase(candidates.begin() + indexRandom);
	return indexLC;
}

// -----------------------------------------------------------------------------------------------------
//  selectCandidateLcPolynom
// -----------------------------------------------------------------------------------------------------
int selectCandidateLcPolynom(vector<int> &candidates)
{
	if (candidates.size()<1)
		return -1;
	int randomValue = (rand() % 100)^2;
	int indexRandom = randomValue * candidates.size() / 10000;
	int indexLC = candidates[indexRandom];
	// remove this item from the list
	candidates.erase(candidates.begin() + indexRandom);
	return indexLC;
}

void loadPresetLC(map<int,int> &presetLC)
{
	std::ifstream filePresetLC("preset_LC.dat");
	int pose1, pose2;

	if (filePresetLC.is_open())
	{
		while (! filePresetLC.eof())
		{
			filePresetLC >> pose1 >> pose2;

			// add entry in the map
			presetLC[pose1] = pose2;

			filePresetLC.ignore(256, '\n');	// to reach end of line
			filePresetLC.peek();				// to update the eof flag (if no empty line)
		}
	}
}

bool Map::detectLoopClosure(const PoseVector	&cameraPoses)
{
	bool returnValue = false;
	Pose		currentPose;
	FrameData *pFrameDataLC = NULL;
	FrameData frameDataCurrent;
	vector <int>		idCandidateLC;
	vector <FrameData*>	bufferFrameDataLC;
	Transformation bestTransformLC;
	Transformation transform;
	int indexBestLC;
	bool foundLoopClosure = false;
	bool insertLoopClosure = false;

	char bufLog[256];
	sprintf(bufLog, "%s/%s", Config::_ResultDirectory.c_str(), "loop_closure.log");
	std::ofstream logLC(bufLog);

	map<int,int> mapPresetLC;

	loadPresetLC(mapPresetLC);

	// loop
	for (int iPose=0; iPose<cameraPoses.size(); iPose++)
	{
		currentPose = cameraPoses[iPose];

		/*if (totalDistance > Config::_LoopClosureDistance ||
			getAngleTransform(currentPose._matrix) > Config::_LoopClosureAngle)*/
		if (true) // TODO - define LC trigger test
		{
			insertLoopClosure = true;
			vector<int> currentCandidates;
			int indexLC;
			int nbSamples = 0;

			// check for LC preset "around" the current pose
			for (int iTest=0; iTest<=currentPose._id; iTest++)
			{
				if (mapPresetLC.find(iTest) != mapPresetLC.end())
				{
					int node = mapPresetLC[iTest];
					cout << "Search indexLC for node " << node << " from " << iTest << "\n";
					mapPresetLC.erase(mapPresetLC.find(iTest));
					currentCandidates.clear();

					// search target node "around" the item given in the map
					for (indexLC=0; indexLC<idCandidateLC.size(); indexLC++)
						if (idCandidateLC[indexLC]>=node)  // candidate list is ordered
						{
							currentCandidates.push_back(indexLC);
							nbSamples = 1;
							break;	// found
						}
				}
			}

			if (foundLoopClosure) {
				// candidate already known
				currentCandidates.clear();
				currentCandidates.push_back(indexBestLC);
				nbSamples = 1;
			}

			if (nbSamples==0) {
				// define random sample list
				nbSamples = Config::_LoopClosureWindowSize;
				if (idCandidateLC.size() < Config::_LoopClosureWindowSize)
					nbSamples = idCandidateLC.size();
				// generate list of indexes where samples will be pickup once
				int sizeLC = idCandidateLC.size()-5;	// ignore last n poses
				for (int i=0; i<sizeLC; i++)
					currentCandidates.push_back(i);
				if (nbSamples>currentCandidates.size())
					nbSamples = currentCandidates.size();
			}

			// check loop closure for every sample
			for (int iSample=0; iSample<nbSamples; iSample++)
			{
				if (currentCandidates.size()==1)
					indexLC = currentCandidates[0];	// candidate already known
				else
					indexLC = selectCandidateLcPolynom(currentCandidates);	// the list is updated!

				cout << "Checking loop closure " << iSample+1 << "/" << nbSamples;
				cout << "\tCandidate frames: " << idCandidateLC[indexLC] << "-" << currentPose._id;
				//cout << "\tTotalDistance=" << totalDistance << "(m)\tAngle=" <<  getAngleTransform(currentPose._matrix) << "(deg)\n";
				cout << "\n";

				// loop closure with frame
				bool validLC = checkLoopClosure(
						idCandidateLC[indexLC],
						currentPose._id,
						*bufferFrameDataLC[indexLC],
						frameDataCurrent,
						transform);

				if (validLC)
				{
					cout << " LOOP CLOSURE DETECTED (" << idCandidateLC[indexLC];
					cout << "-" << currentPose._id <<  ") \n";
					cout << "Ratio: " << transform._ratioInliers << "\n";
					logLC << " LOOP CLOSURE DETECTED (" << idCandidateLC[indexLC];
					logLC << "-" << currentPose._id <<  ") \n";
					logLC << "Ratio: " << transform._ratioInliers << "\n";

					// looking for a better loop closure
					if (transform._ratioInliers >= bestTransformLC._ratioInliers)
					{
						// a better loop closure is found
						foundLoopClosure = true;
						// it may be improved, don't insert it yet
						insertLoopClosure = false;
						// keep the loop closure info
						bestTransformLC = transform;
						indexBestLC = indexLC;
					}
				}
			}
			//cout << "----------------------------------------------------------------------------\n";
		}

		// trigger the loop closure unless a better one has just been found
		if (foundLoopClosure && insertLoopClosure)
		{
			cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
			cout << "Adding edge for Loop Closure";
			cout << " between vertex " << bestTransformLC._idOrig << " and " << bestTransformLC._idDest << "\n";
			cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
			logLC << "Adding edge for Loop Closure";
			logLC << " between vertex " << bestTransformLC._idOrig << " and " << bestTransformLC._idDest << "\n";

			// add the edge = new constraint
			_graphOptimizer.addEdge(bestTransformLC);
			returnValue = true;

			// remove the candidates included in the ID range of the loop closure
			// this supposes the ID are sorted
			//cout << "Removing candidates... " ;
			/*int sizeLC = idCandidateLC.size();
			for (int indexLC=sizeLC-1; indexLC>=0; indexLC--)
			{
				if (idCandidateLC[indexLC] >= bestTransformLC._idOrig &&
					idCandidateLC[indexLC] <= bestTransformLC._idDest)
				{
					//cout << indexLC << ": id=" << idCandidateLC[indexLC] << "\n";
					// free loop closure data
					if (bufferFrameDataLC[indexLC]!=NULL)
						delete bufferFrameDataLC[indexLC];
					// remove candidate
					idCandidateLC.erase(idCandidateLC.begin()+indexLC);
					bufferFrameDataLC.erase(bufferFrameDataLC.begin()+indexLC);
				}
			}
			cout << "\n";
			cout << "Size Candidate List Before: " << sizeLC << " After: " << idCandidateLC.size() << "\n";
			logLC << "Size Candidate List Before: " << sizeLC << " After: " << idCandidateLC.size() << "\n";*/

			// reset loop closure
			foundLoopClosure = false;
			bestTransformLC._idOrig = -1;
			bestTransformLC._idDest = -1;
			bestTransformLC._ratioInliers = 0;
			bestTransformLC._matrix = Eigen::Matrix4f::Identity();
			//totalDistance = 0;
		}

		// insert as candidate for loop closure
		pFrameDataLC = new FrameData;
		bufferFrameDataLC.push_back(pFrameDataLC);
		idCandidateLC.push_back(currentPose._id);
	}

	// pending loop closure
	if (foundLoopClosure)
	{
		cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
		cout << "Adding edge for Loop Closure";
		cout << " between vertex " << bestTransformLC._idOrig << " and " << bestTransformLC._idDest << "\n";
		cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
		logLC << "Adding edge for Loop Closure";
		logLC << " between vertex " << bestTransformLC._idOrig << " and " << bestTransformLC._idDest << "\n";
		// add new constraint
		_graphOptimizer.addEdge(bestTransformLC);
		returnValue = true;
	}

	// free loop closure data
	for (int indexLC=0; indexLC<bufferFrameDataLC.size(); indexLC++)
		delete bufferFrameDataLC[indexLC];

	return 	returnValue;
}

// -----------------------------------------------------------------------------------------------------
//  build
// -----------------------------------------------------------------------------------------------------
void Map::build()
{
	PoseVector	cameraPoses;
	Pose		currentPose;
	TransformationVector sequenceKeyTransform;
	Transformation keyTransform;
	double totalDistance = 0.0;

	// initialize the graph
    _graphOptimizer.initialize();
    
    if (_sequenceTransform.size()>0)
    {
		// -------------------------------------------------------------------------------------------
		//  origin
		// -------------------------------------------------------------------------------------------
    	currentPose._matrix = Eigen::Matrix4f::Identity();
    	currentPose._id = _sequenceTransform[0]._idOrig;
    	cameraPoses.push_back(currentPose);
		// add vertex for the initial pose
		_graphOptimizer.addVertex(currentPose);
		keyTransform._matrix = Eigen::Matrix4f::Identity();
		keyTransform._idOrig = _sequenceTransform[0]._idOrig;
		cout << "Start sequence at " << keyTransform._idOrig << "\n";
		
		// -------------------------------------------------------------------------------------------
		//  loop on the global sequence of transformations
		// -------------------------------------------------------------------------------------------
		for (int iTransform=0; iTransform<_sequenceTransform.size(); iTransform++)
		{
			// compute new position of the camera, by cumulating the inverse transforms (right side)
			currentPose._matrix = currentPose._matrix * _sequenceTransform[iTransform]._matrix.inverse();
			currentPose._id = _sequenceTransform[iTransform]._idDest;
			
			// update key transformation by cumulating the transforms (left side)
			keyTransform._matrix = _sequenceTransform[iTransform]._matrix * keyTransform._matrix;
			//cout << keyTransform._idOrig << "-" << _sequenceTransform[iTransform]._idDest << "\n";
			
			if (isKeyTransform(keyTransform._matrix) || iTransform==_sequenceTransform.size()-1)
			{
				// -------------------------------------------------------------------------------------------
				//  new keynode: define camera position in the graph
				// -------------------------------------------------------------------------------------------
				totalDistance += getDistanceTransform(keyTransform._matrix);
				cout << keyTransform._idOrig << "-" << _sequenceTransform[iTransform]._idDest << ": ";
				//cout << "Distance=" << getDistanceTransform(keyTransform._matrix) << "(m)\tAngle=" << getAngleTransform(keyTransform._matrix) << "(deg)\n";
				double angle[3];
				getAngleTransform(keyTransform._matrix, angle);
				cout << "Distance=" << getDistanceTransform(keyTransform._matrix);
				cout << "(m)\tAngles=" << angle[0]/PI180 << "," << angle[1]/PI180 << "," << angle[2]/PI180 << " (deg)\n";

				// add a vertex = current camera pose
				cameraPoses.push_back(currentPose);
				_graphOptimizer.addVertex(currentPose);

				// add an edge = new constraint relative to the key transform
				keyTransform._idDest = currentPose._id;
				sequenceKeyTransform.push_back(keyTransform);
				_graphOptimizer.addEdge(keyTransform);

				// remap from this position (relative key transform)
				keyTransform._matrix = Eigen::Matrix4f::Identity();
				keyTransform._idOrig = currentPose._id;
			}
		}

		_graphOptimizer.save("graph_initial.g2o");

		// build initial point cloud
		if (Config::_PcdGenerateInitial)
			PointCloud::generatePCD(cameraPoses, "cloud_initial");

		// -------------------------------------------------------------------------------------------
		//  loop closure
		// -------------------------------------------------------------------------------------------
		if (! detectLoopClosure(cameraPoses))
			printf("NO LOOP CLOSURE DETECTED!\n");
		else
		{
			// new constraints in initial graph
			_graphOptimizer.save("graph_initial.g2o");

			// -------------------------------------------------------------------------------------------
			//  optimize the graph
			// -------------------------------------------------------------------------------------------
			_graphOptimizer.optimize();
			_graphOptimizer.save("graph_optimized.g2o");

			// extract the updated camera positions from the optimized graph
			_graphOptimizer.extractAllPoses(cameraPoses);

			// generate optimized point cloud
			if (Config::_PcdGenerateOptimized)
				PointCloud::generatePCD(cameraPoses, "cloud_optimized");
		}

		savePoses(cameraPoses, "poses.dat");
    }
}

// -----------------------------------------------------------------------------------------------------
//  initSequence
// -----------------------------------------------------------------------------------------------------
void Map::initSequence()
{
	char buf[256];
	sprintf(buf, "%s/transfo.dat", Config::_ResultDirectory.c_str());
	_fileTransformOut.open(buf);

	_sequenceTransform.clear();

	// reset stats log
	sprintf(buf, "%s/stats.log", Config::_ResultDirectory.c_str());
	std::ofstream fileStats(buf);
}


// -----------------------------------------------------------------------------------------------------
//  addFrames
// -----------------------------------------------------------------------------------------------------
bool Map::addFrames(int frameID1, int frameID2, Transformation &transform)
{
	bool match = false;

	// match frame to frame (current with previous)
	match = computeTransformation(
			frameID1,
			frameID2,
			_bufferFrameData1,
			_bufferFrameData2,
			transform);

	if (match)	{
		// valid transform
		_sequenceTransform.push_back(transform);

		// archive transform
		if (_fileTransformOut.is_open())
		{
			_fileTransformOut << transform._idOrig << " " << transform._idDest << " ";
			for (int irow=0; irow<4; irow++)
				for (int icol=0; icol<4; icol++)
					_fileTransformOut << transform._matrix(irow,icol) << " ";
			_fileTransformOut << std::endl;
		}

		// text display of quality
		int c;
		for (c=0; c<transform._qualityScore; c++)
			cout << '+';
		while (c<100){
			cout << '-';
			c++;
		}
		cout << "\n";

		_display.showFeatures(_bufferFrameData1, _bufferFrameData2, transform._qualityScore);

		// free data
		_bufferFrameData1.releaseData();
		// reassign the last frame to avoid reloading all the data twice
		_bufferFrameData1.assignData(_bufferFrameData2);
	}
	else { // out of sync !
		_display.showOutOfSync(_bufferFrameData1, _bufferFrameData2);

		// invalid transformation
		// empty buffers - data has to be reloaded
		_bufferFrameData1.releaseData();
		_bufferFrameData2.releaseData();
	}

	return match;
}

// -----------------------------------------------------------------------------------------------------
//  addSequence
// -----------------------------------------------------------------------------------------------------
void Map::addSequence(std::vector<int> &sequenceFramesID)
{
	if (sequenceFramesID.size()>=2)
	{
		Transformation transform;
		int indexLastFrame;
		int stepFrame = Config::_DataInRatioFrame;

		indexLastFrame = 0;
		for (int iFrame=stepFrame; iFrame<sequenceFramesID.size(); iFrame+=stepFrame)
		{
			if (! addFrames(
					sequenceFramesID[indexLastFrame],
					sequenceFramesID[iFrame],
					transform))
			{
				// no valid transform
				std::cerr << transform._idOrig << "-" << transform._idDest << ": ";
				std::cerr << "No valid transformation!!";
				std::cerr << "\tRatio=" << transform._ratioInliers*100 << "% ";
				std::cerr << "\tResync attempt #" <<  Config::_DataInRatioFrame-stepFrame+1 << "\n";
				// try to lower step
				stepFrame--;
				if (stepFrame>0)
				{
					// go back
					iFrame = indexLastFrame;
					continue;	// skip current frame
				}
				else
				{
					if (! Config::_MatchingAllowInvalid)
						break;		// abort
					std::cerr << "INVALID TRANSFORMATION RECORDED! Frames ";
					std::cerr << transform._idOrig << "-" << transform._idDest << "\n";
				}
			}
			
			indexLastFrame = iFrame;
			stepFrame = Config::_DataInRatioFrame;
		}
	}
	
}

// -----------------------------------------------------------------------------------------------------
//  restoreSequence
// -----------------------------------------------------------------------------------------------------
void Map::restoreSequence(int minFrameID, int maxFrameID)
{
	Transformation transfo;
	char buf[256];
	sprintf(buf, "%s/transfo.dat", Config::_ResultDirectory.c_str());
	std::ifstream fileTransfo(buf);
	int check=-1;
	
	_sequenceTransform.clear();

	printf("Restoring sequence range %d-%d\n", minFrameID, maxFrameID);
	if (fileTransfo.is_open())
	{
		// read sequence archive
		while (! fileTransfo.eof())
		{
			// archive transfo
			fileTransfo >> transfo._idOrig >> transfo._idDest;
			//std::cout << "Restore Matrix "<< transfo._idOrig << "-" << transfo._idDest << ":\n";
			for (int irow=0; irow<4; irow++)
				for (int icol=0; icol<4; icol++)
					fileTransfo >> transfo._matrix(irow,icol);

			// check there is no hole in the sequence, it should be continuous
			if (check==-1)
				check = transfo._idDest;
			else {
				if (transfo._idOrig != check) {
					printf("Invalid sequence!! Found %d, awaited %d.\nCheck file %s\n", transfo._idOrig, check, buf);
					return;
				}
				check = transfo._idDest;
			}

			//std::cout << transfo._matrix << std::endl;
			if (transfo._idOrig >= minFrameID && (transfo._idDest <= maxFrameID || maxFrameID<0))
				_sequenceTransform.push_back(transfo);

			fileTransfo.ignore(256, '\n');	// to reach end of line
			fileTransfo.peek();				// to update the eof flag (if no empty line)
		}
	}
}


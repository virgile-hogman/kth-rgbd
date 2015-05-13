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

#include <Eigen/Geometry>

#include "Config.h"
#include "CommonTypes.h"
#include "Matching.h"
#include "FrameData.h"
#include "PointCloud.h"
#include "CameraDevice.h"
#include "TimeTracker.h"
#include "Sequence.h"

#include <iostream>
#include <vector>

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
//  loadPoses - !!!NOT TESTED YET!!! poses are restored via graph from g2o file -
// -----------------------------------------------------------------------------------------------------
void loadPoses(PoseVector &poses, const char *filename)
{
	char buf[256];
	sprintf(buf, "%s/%s", Config::_PathDataProd.c_str(), filename);
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
	sprintf(buf, "%s/%s", Config::_PathDataProd.c_str(), filename);
	std::ofstream filePoses(buf);

	//filePoses << "#Num\tFrameID\tMatrix rows,cols";
	for (int i=0; i<poses.size(); i++)
	{
		filePoses << poses[i]._id;
		for (int r=0; r<3; r++)
			for (int c=0; c<4; c++)
				filePoses << "\t" << poses[i]._matrix(r,c);
		filePoses << "\n";
	}
}

// -----------------------------------------------------------------------------------------------------
//  regeneratePCD
// -----------------------------------------------------------------------------------------------------
void Sequence::regeneratePCD()
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
//  selectCandidateLcUniform
// -----------------------------------------------------------------------------------------------------
int selectCandidateLcUniform(vector<int> &candidates)
{
	if (candidates.size()<1)
		return -1;
	// pickup a candidate randomly with uniform distribution
	int indexRandom = rand() % candidates.size();
	int indexLC = candidates[indexRandom];
	// remove this item from the list
	candidates.erase(candidates.begin() + indexRandom);
	return indexLC;
}

// -----------------------------------------------------------------------------------------------------
//  selectCandidateLcLinear
// -----------------------------------------------------------------------------------------------------
int selectCandidateLcLinear(vector<int> &candidates)
{
	if (candidates.size()<1)
		return -1;
	// define an arithmetic progression
	int sumAll = candidates.size()*(candidates.size()+1)/2;
	int valRandom = rand() % sumAll;	// rand value 0..sum-1
	//printf("(%d/%d)", valRandom, sumAll);

	int indexRandom = 0;	// start index
	int sum = candidates.size()-1;	// cumulated sum 0 based
	while (sum<valRandom && indexRandom<candidates.size()) {
		sum += candidates.size()-indexRandom-1;
		indexRandom++;
	}

	int indexLC = candidates[indexRandom];
	// remove this item from the list
	candidates.erase(candidates.begin() + indexRandom);
	return indexLC;
}

// -----------------------------------------------------------------------------------------------------
//  selectCandidateLcQuadatric
// -----------------------------------------------------------------------------------------------------
int selectCandidateLcQuadatric(vector<int> &candidates)
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

// -----------------------------------------------------------------------------------------------------
//  testRandom
// -----------------------------------------------------------------------------------------------------
void testRandom()
{
	vector<int> v;
	int total[10];
	int nbVals=10;

	for (int i=0; i<nbVals; i++)
		total[i]=0;
	for (int k=0; k<200; k++) {
		v.clear();
		for (int i=0; i<nbVals; i++)
			v.push_back(i);
		printf("TEST %d --> ", k);
		for (int j=0; j<5; j++) {
			int index=selectCandidateLcLinear(v);
			printf(" %d/%ld # ", index, v.size());
			total[index]++;
		}
		printf("\n");
	}
	printf("Totals: ");
	for (int i=0; i<nbVals; i++)
		printf("%d ", total[i]);
	printf("\n");
}

// -----------------------------------------------------------------------------------------------------
//  loadPresetLC
// -----------------------------------------------------------------------------------------------------
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

bool Sequence::detectLoopClosure(const PoseVector	&cameraPoses)
{
	bool returnValue = false;
	Pose		currentPose;
	FrameData *pFrameDataLC = NULL;
	FrameData frameDataCurrent;
	vector <int>		idCandidateLC;
	vector <FrameData*>	bufferFrameDataLC;
	Transformation bestTransformLC;	// initial ratio_inliers=0
	Transformation transform;
	int indexBestLC;
	bool foundLoopClosure = false;
	bool insertLoopClosure = false;

	char bufLog[256];
	sprintf(bufLog, "%s/%s", Config::_PathDataProd.c_str(), "loop_closure.log");
	std::ofstream logLC(bufLog);

	map<int,int> mapPresetLC;

	loadPresetLC(mapPresetLC);

	// main loop over the full set of poses
	for (int iPose=0; iPose<cameraPoses.size(); iPose++)
	{
		list<int> samples;
		int nbSamples = 0;
		int indexLC;
		insertLoopClosure = true;

		currentPose = cameraPoses[iPose];

		// check for LC preset "around" the current pose
		for (int iTest=0; iTest<=currentPose._id; iTest++)
		{
			if (mapPresetLC.find(iTest) != mapPresetLC.end())
			{
				int node = mapPresetLC[iTest];
				cout << "Search indexLC for node " << node << " from " << iTest << "\n";
				mapPresetLC.erase(mapPresetLC.find(iTest));
				samples.clear();

				// search target node "around" the item given in the map
				for (indexLC=0; indexLC<idCandidateLC.size(); indexLC++)
					if (idCandidateLC[indexLC]>=node)  // candidate list is ordered
					{
						samples.push_back(indexLC);
						break;	// found
					}
			}
		}

		if (foundLoopClosure) {
			// candidate already known
			samples.clear();
			samples.push_back(indexBestLC);
		}

		nbSamples = samples.size();
		if (nbSamples==0) {
			// define random sample list
			vector<int> randomSamples;
			nbSamples = Config::_LoopClosureWindowSize;
			if (idCandidateLC.size() < Config::_LoopClosureWindowSize)
				nbSamples = idCandidateLC.size();
			// generate list of indexes where samples will be taken from
			int sizeLC = idCandidateLC.size() - Config::_LoopClosureExcludeLast;	// ignore last n poses
			for (int i=0; i<sizeLC; i++)
				randomSamples.push_back(i);
			if (nbSamples>randomSamples.size())
				nbSamples = randomSamples.size();

			for (int iSample=0; iSample<nbSamples; iSample++) {
				indexLC = selectCandidateLcLinear(randomSamples);	// the list is updated!
				samples.push_back(indexLC);
			}
			// order the list of samples
			samples.sort();
		}

		// check loop closure for every sample
		while (! samples.empty())
		{
			indexLC = samples.front();
			samples.pop_front();

			cout << "Checking loop closure " << nbSamples-samples.size() << "/" << nbSamples;
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

			// free RGBD data in frame buffer
			bufferFrameDataLC[indexLC]->releaseImageRGBD();

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
			bestTransformLC.reset();
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
//  buildMap
// -----------------------------------------------------------------------------------------------------
void Sequence::buildMap()
{
	PoseVector	cameraPoses;
	Pose		currentPose;
	TransformationVector sequenceKeyTransform;
	Transformation keyTransform;
	double totalDistance = 0.0;
    Eigen::Matrix3f initialRotation = Eigen::Matrix3f::Identity();

    if (_sequenceTransform.size()>0)
    {
    	// initialize the graph
        _graphOptimizer.initialize();

		// -------------------------------------------------------------------------------------------
		//  origin
		// -------------------------------------------------------------------------------------------
    	initialRotation = Eigen::AngleAxisf(Config::_MapInitialAngle[0] * M_PI/180.0, Eigen::Vector3f::UnitX())
				* Eigen::AngleAxisf(Config::_MapInitialAngle[1] * M_PI/180.0, Eigen::Vector3f::UnitY())
				* Eigen::AngleAxisf(Config::_MapInitialAngle[2] * M_PI/180.0, Eigen::Vector3f::UnitZ());
    	currentPose._matrix = Eigen::Matrix4f::Identity();
    	currentPose._matrix.block(0,0,3,3) = initialRotation;
    	currentPose._matrix(0,3) = Config::_MapInitialCoord[0];
    	currentPose._matrix(1,3) = Config::_MapInitialCoord[1];
    	currentPose._matrix(2,3) = Config::_MapInitialCoord[2];
    	cout << "Initial pose:\n" << currentPose._matrix << "\n";
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

			_display.processEvent();
		}

		_graphOptimizer.save("graph_initial.g2o");
		savePoses(cameraPoses, "poses_initial.dat");
		_graphOptimizer.extractAllPoses(cameraPoses);

		// build initial point cloud
		if (Config::_PcdGenerateInitial)
			PointCloud::generatePCD(cameraPoses, "cloud_initial");

		// -------------------------------------------------------------------------------------------
		//  loop closure
		// -------------------------------------------------------------------------------------------
		if (! detectLoopClosure(cameraPoses))
			cout << "NO LOOP CLOSURE DETECTED!\n";
		else
		{
			// new constraints in initial graph, overwrite
			_graphOptimizer.save("graph_initial.g2o");
			// optimize the graph
			optimizeMap();
		}
    }
}

// -------------------------------------------------------------------------------------------
//  OptimizeMap
// -------------------------------------------------------------------------------------------
void Sequence::optimizeMap() {
	PoseVector	cameraPoses;

	int ret = _graphOptimizer.optimize();
	if (ret <= 0)
		cout << "Failed to optimize the graph! err=" << ret << "\n";
	else {
		_graphOptimizer.save("graph_optimized.g2o");

		// extract the updated camera positions from the optimized graph
		_graphOptimizer.extractAllPoses(cameraPoses);
		savePoses(cameraPoses, "poses_optimized.dat");

		// generate optimized point cloud
		if (Config::_PcdGenerateOptimized)
			PointCloud::generatePCD(cameraPoses, "cloud_optimized");
	}
}

// -------------------------------------------------------------------------------------------
//  restoreInitialGraph
// -------------------------------------------------------------------------------------------
void Sequence::restoreInitialGraph() {
    _graphOptimizer.initialize();
    _graphOptimizer.load("graph_initial.g2o");
}

// -----------------------------------------------------------------------------------------------------
//  startTransform
// -----------------------------------------------------------------------------------------------------
void Sequence::startTransform()
{
	char buf[256];
	sprintf(buf, "%s/transfo.dat", Config::_PathDataProd.c_str());
	_fileTransformOut.open(buf);

	_sequenceTransform.clear();

	// reset stats log
	sprintf(buf, "%s/stats.log", Config::_PathDataProd.c_str());
	std::ofstream fileStats(buf);
}

// -----------------------------------------------------------------------------------------------------
//  stop
// -----------------------------------------------------------------------------------------------------
void Sequence::stopTransform()
{
	// close windows
	_display.hide();
	_fileTransformOut.close();
}

// -----------------------------------------------------------------------------------------------------
//  addFramesTransform
// -----------------------------------------------------------------------------------------------------
bool Sequence::addFramesTransform(int frameID1, int frameID2, Transformation &transform, int &keywait)
{
	bool match = false;
	keywait = -1;

	// match frame to frame (current with previous)
	match = computeTransformation(
			frameID1,
			frameID2,
			_bufferFrame1,
			_bufferFrame2,
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

		keywait = _display.showFeatures(_bufferFrame1.getImage(), _bufferFrame2.getImage(), transform._qualityScore);

		// free data
		_bufferFrame1.releaseData();
		// reassign the last frame to avoid reloading all the data twice
		_bufferFrame1.assignData(_bufferFrame2);
	}
	else { // out of sync !
		keywait = _display.showOutOfSync(_bufferFrame1.getImage(), _bufferFrame2.getImage());

		// invalidate current buffer
		_bufferFrame2.releaseData();
	}

	return match;
}


// -----------------------------------------------------------------------------------------------------
//  restoreTransformations
// -----------------------------------------------------------------------------------------------------
void Sequence::restoreTransformations(int minFrameID, int maxFrameID)
{
	Transformation transfo;
	char buf[256];
	sprintf(buf, "%s/transfo.dat", Config::_PathDataProd.c_str());
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

// -----------------------------------------------------------------------------------------------------
//  reloadFrames
// -----------------------------------------------------------------------------------------------------
bool Sequence::reloadFrames(int min, int max)
{
	vector<int> sequenceFramesID;

	// build the list of FrameID from the default path
	FrameData::GetFrameList(Config::_PathFrameSequence.c_str(), min, max, sequenceFramesID);

	if (sequenceFramesID.size()>=2)
	{
		Transformation transform;
		int indexLastFrame;
		int stepFrame = Config::_DataInRatioFrame;

	    // start sequence
		this->startTransform();

		indexLastFrame = 0;
		for (int iFrame=stepFrame; iFrame<sequenceFramesID.size(); iFrame+=stepFrame)
		{
			// add current couple of frames
			int keyb;
			if (! addFramesTransform(
					sequenceFramesID[indexLastFrame],
					sequenceFramesID[iFrame],
					transform,
					keyb))
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
	    // stop sequence
		this->stopTransform();

		return true;
	}
	return false;
}

// -----------------------------------------------------------------------------------------------------
//  record
// -----------------------------------------------------------------------------------------------------
void Sequence::recordFrames(bool buildMap)
{
	CameraDevice cameraKinect;
	TimeTracker tm;

    if (cameraKinect.connect())
    {
    	int frameID = 0;
    	Transformation transform;
    	bool abort = false;
    	bool keyhit = false;
        char keyval;
        int keywait;
        FrameData frame1, frame2;

        frame1.createImageRGB();
        frame2.createImageRGB();

        // working buffers reused for each frame (just to avoid reallocate the arrays each time)
        IplImage* imgBufferDepth = cvCreateImage(cvSize(NBPIXELS_WIDTH, NBPIXELS_HEIGHT),IPL_DEPTH_8U,3);
        IplImage* imgRGB = cvCreateImage(cvSize(NBPIXELS_WIDTH, NBPIXELS_HEIGHT),IPL_DEPTH_8U,3);

   	  	printf("*** PRESS ANY KEY TO START // ENTER TO STOP // ESC TO ABORT ***\n");

    	// generate 1st frame
        while(!abort && !keyhit) {
        	if (! cameraKinect.generateFrame(imgRGB, imgBufferDepth))
        		break;

			// recopy to data
        	frame1.setFrameID(frameID);
        	frame1.copyImageDepth(imgBufferDepth);
        	frame1.copyImageRGB(imgRGB);

			// compute and draw features
        	frame1.computeFeatures();
        	frame1.drawFeatures();
			// display RGB with features and depth
			keywait = _display.showPreview(frame1.getImage(), imgBufferDepth);
			// check keyhit first from GUI otherwise from console through camera/openni
        	if (keywait>0) {
        		keyhit = true;
        		keyval = (char)keywait;
        	}	
        	else
				keyhit = cameraKinect.getUserInput(keyval);
        	if (keyhit>0) {
        		printf("\n");
        		if (keyval==27) // ESC
        			abort = true;
        		else {
        			// save RGBD frames
        			cameraKinect.saveImageRGB(imgRGB, frameID);
        			cameraKinect.saveImageDepth(imgBufferDepth, frameID);
        		}
        	}
		}
		// imgRGB not needed anymore, stored directly in frame data
        cvReleaseImage(&imgRGB);
        imgRGB = NULL;

        if (!abort && keyhit) {
            // start sequence
    		tm.start();
			frameID++;
			bool validate = false;

			this->startTransform();
			while (!abort && !validate) {
				// generate new frame
				if (! cameraKinect.generateFrame(frame2.getImage(), imgBufferDepth))
					break;

				if (!validate && !abort)	{
					// save RGBD frames
        			cameraKinect.saveImageRGB(frame2.getImage(), frameID);
        			cameraKinect.saveImageDepth(imgBufferDepth, frameID);
					// associate with previous
					if (! addFramesTransform(frameID-1, frameID, transform, keywait)) {
						printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
						printf("!!! LOW QUALITY !!! LOST SYNCHRO !!!\n");
						printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
						printf("Last Frame (%d) rejected\n", frameID);
					}
					else
						frameID++;

					// check keyhit first from GUI otherwise from console through camera/openni
					if (keywait>0) {
						keyhit = true;
						keyval = (char)keywait;
					}	
					else
						keyhit = cameraKinect.getUserInput(keyval);

					if (keyhit) {
		        		printf("\n");
						switch (keyval) {
						case 27:	// ESC
							abort = true;
							break;
						case 13:	// CR
						case 10:	// LF
							validate = true;
							break;
						}
					}						
				}
			}

			tm.stop();
			printf("Acquisition and matching duration time: %d(ms)\n", tm.duration());
			this->stopTransform();

			cvReleaseImage(&imgBufferDepth);
			frame1.releaseData();
	        frame2.releaseData();

	        if (abort)
	        	printf("Aborted.\n");

	        if (buildMap && validate)
				this->buildMap();
        }

    	cameraKinect.disconnect();
    }
}


#include <Eigen/Geometry>

// PCL includes
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/common/transform.h"

#include "Config.h"
#include "CameraDevice.h"
#include "CommonTypes.h"
#include "Matching.h"
#include "FrameData.h"
#include "Map.h"

#include <iostream>

using namespace std;


bool getFramePointCloud(int frameID, pcl::PointCloud<pcl::PointXYZRGB> &pointCloud)
{
	FrameData frameData;
	if (!frameData.loadImage(frameID))
	{
		pointCloud.points.clear();		
		return false;
	}
	if (!frameData.loadDepthData())
	{
		pointCloud.points.clear();		
		return false;
	}
	
	// allocate the point cloud buffer
	pointCloud.width = NBPIXELS_WIDTH;
	pointCloud.height = NBPIXELS_HEIGHT;
	pointCloud.points.clear();
	pointCloud.points.reserve(NBPIXELS_WIDTH*NBPIXELS_HEIGHT);	// memory preallocation
	//pointCloud.points.resize(NBPIXELS_WIDTH*NBPIXELS_HEIGHT);
	
	//printf("Generating point cloud frame %d\n", frameID);
		
	float constant = 0.001 / CameraDevice::_FocalLength;
	unsigned int rgb; 
	int depth_index = 0;
	IplImage *img = frameData.getImage();
	for (int ind_y =0; ind_y < NBPIXELS_HEIGHT; ind_y++)
	{
		for (int ind_x=0; ind_x < NBPIXELS_WIDTH; ind_x++, depth_index++)
		{
			//pcl::PointXYZRGB& pt = pointCloud(ind_x,ind_y);
			TDepthPixel depth = frameData.getDepthData()[depth_index];
			if (depth != 0 ) 
			{
				pcl::PointXYZRGB pt;
				
				// locate point in meters
				pt.z = (ind_x - NBPIXELS_X_HALF) * depth * constant;
				pt.y = (NBPIXELS_Y_HALF - ind_y) * depth * constant;
				pt.x = depth * 0.001 ; // given depth values are in mm

				unsigned char b = ((uchar *)(img->imageData + ind_y*img->widthStep))[ind_x*img->nChannels + 0];
				unsigned char g = ((uchar *)(img->imageData + ind_y*img->widthStep))[ind_x*img->nChannels + 1];
				unsigned char r = ((uchar *)(img->imageData + ind_y*img->widthStep))[ind_x*img->nChannels + 2];
				rgb = (((unsigned int)r)<<16) | (((unsigned int)g)<<8) | ((unsigned int)b);
				pt.rgb = *reinterpret_cast<float*>(&rgb);
				pointCloud.push_back(pt);
			}
		}
	}
	
	return true;
}

// -----------------------------------------------------------------------------------------------------
//  subsamplePointCloud
// -----------------------------------------------------------------------------------------------------
void subsamplePointCloud(pcl::PointCloud<pcl::PointXYZRGB> &pointCloud, int ratioKeep)
{
	pcl::PointCloud<pcl::PointXYZRGB> cloudTemp;

	if (ratioKeep<0 || ratioKeep>=100)
		return;

	for (int i=0; i<pointCloud.size(); i++)
	{
		if (rand()%100 < ratioKeep)
			cloudTemp.push_back(pointCloud.points[i]);
	}
	pointCloud = cloudTemp;
	/*
	const float voxel_grid_size = 0.005;
	pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;  
	vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
	vox_grid.setInputCloud (pointCloud());
	vox_grid.filter(pointCloud);*/
	//cout << "Size: " << pointCloud.size() << " points after subsampling " << ratioKeep << "%" << std::endl;
}

// -----------------------------------------------------------------------------------------------------
//  generateMapPCD
// -----------------------------------------------------------------------------------------------------
void generateMapPCD(const PoseVector &cameraPoses, const char *filenamePCD)
{
	char buf_full[256];
	pcl::PointCloud<pcl::PointXYZRGB> cloudFull;
	pcl::PointCloud<pcl::PointXYZRGB> cloudFrame;
	pcl::PointCloud<pcl::PointXYZRGB> cloudFrameTransformed;
	int nbPCD=0;
	
	for (int iPose=0; iPose<cameraPoses.size(); iPose++)
	{
		if (iPose % Config::_PcdRatioFrame != 0)
			continue;	// skip frame

		cout << "----------------------------------------------------------------------------\n";
		cout << "Append point cloud frame #" << cameraPoses[iPose]._id << " (pose " << iPose+1 << "/" << cameraPoses.size() << ")\n";
		cout << cameraPoses[iPose]._matrix << std::endl;

		getFramePointCloud(cameraPoses[iPose]._id, cloudFrame);

		// subsample frame
		subsamplePointCloud(cloudFrame, Config::_PcdRatioKeepSubsample);
		
		// apply transformation to the point cloud
		pcl::getTransformedPointCloud(
				cloudFrame,
				Eigen::Affine3f(cameraPoses[iPose]._matrix),
				cloudFrameTransformed); 
		
		// apend transformed point cloud
		cloudFull += cloudFrameTransformed;
		cout << " Total Size: " << cloudFull.size() << " points (";
		cout <<  cloudFull.size()*100/Config::_PcdMaxNbPoints << "% of PCD capacity)." << std::endl;
		
		if (cloudFull.size() > Config::_PcdMaxNbPoints)
		{
			// max size reached
			cout << "Saving global point cloud binary...\n";
			sprintf(buf_full, "%s/%s_%02d.pcd", Config::_ResultDirectory.c_str(), filenamePCD, nbPCD++);
			cout << "File: " << buf_full << "\n";
			pcl::io::savePCDFile(buf_full, cloudFull, true);
			// bug in PCL - the binary file is not created with the good rights!
			char bufsys[256];
			sprintf(bufsys, "chmod a+rw %s", buf_full);
			system(bufsys);
			cloudFull.points.clear();
		}
	}
	if (cloudFull.size()>0)
	{
		// subsample final point cloud
		//subsamplePointCloud(cloudFull, Config::_PcdRatioKeepSubsample);
		
		cout << "Saving global point cloud binary...\n";
		if (nbPCD>0)
			sprintf(buf_full, "%s/%s_%02d.pcd", Config::_ResultDirectory.c_str(), filenamePCD, nbPCD);
		else
			sprintf(buf_full, "%s/%s.pcd", Config::_ResultDirectory.c_str(), filenamePCD);
		cout << "File: " << buf_full << "\n";
		pcl::io::savePCDFile(buf_full, cloudFull, true);
		// bug in PCL - the binary file is not created with the good rights!
		char bufsys[256];
		sprintf(bufsys, "chmod a+rw %s", buf_full);
		system(bufsys);
	}
}

double getDistanceTransform(const Eigen::Matrix4f &transformMat)
{
	Eigen::Vector3d translation(transformMat(0,3), transformMat(1,3), transformMat(2,3));
	return translation.norm();
}

#define PI 3.14159265
double getAngleTransform(const Eigen::Matrix4f &transformMat)
{
	// convert angle to degrees (should not be multiple of PI!)
	return acos((transformMat(0,0)+transformMat(1,1)+transformMat(2,2) - 1) /2) * 180.0 / PI;
}

bool isKeyTransform(const Eigen::Matrix4f &transformMat)
{
	return (getAngleTransform(transformMat) > Config::_MapNodeAngle ||
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
		generateMapPCD(cameraPoses, "cloud_initial");

	//  optimized graph
	_graphOptimizer.load("graph_optimized.g2o");

	// extract the updated camera positions from the optimized graph
	_graphOptimizer.extractAllPoses(cameraPoses);

	// generate optimized point cloud
	if (Config::_PcdGenerateOptimized)
		generateMapPCD(cameraPoses, "cloud_optimized");
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

void Map::detectLoopClosure(const PoseVector	&cameraPoses)
{
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

			// check if LC preset
			if (mapPresetLC.find(currentPose._id) != mapPresetLC.end())
			{
				int node = mapPresetLC[currentPose._id];
				cout << "Search indexLC for node " << node << " from " << currentPose._id << "\n";
				for (indexLC=0; indexLC<idCandidateLC.size(); indexLC++)
					if (idCandidateLC[indexLC]==node)
					{
						currentCandidates.push_back(indexLC);
						nbSamples = 1;
						break;	// found
					}
			}
			else if (foundLoopClosure)
			{
				// candidate already known
				currentCandidates.push_back(indexBestLC);
				nbSamples = 1;
			}
			else
			{
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
	}

	// free loop closure data
	for (int indexLC=0; indexLC<bufferFrameDataLC.size(); indexLC++)
		delete bufferFrameDataLC[indexLC];
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
			cout << keyTransform._idOrig << "-" << _sequenceTransform[iTransform]._idDest << "\r";
			
			if (isKeyTransform(keyTransform._matrix) || iTransform==_sequenceTransform.size()-1)
			{
				// -------------------------------------------------------------------------------------------
				//  new keynode: define camera position in the graph
				// -------------------------------------------------------------------------------------------
				totalDistance += getDistanceTransform(keyTransform._matrix);
				cout << "Distance=" << getDistanceTransform(keyTransform._matrix) << "(m)\tAngle=" << getAngleTransform(keyTransform._matrix) << "(deg)\n";

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
			generateMapPCD(cameraPoses, "cloud_initial");

		// -------------------------------------------------------------------------------------------
		//  loop closure
		// -------------------------------------------------------------------------------------------
		detectLoopClosure(cameraPoses);
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
			generateMapPCD(cameraPoses, "cloud_optimized");

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
}


// -----------------------------------------------------------------------------------------------------
//  addFrames
// -----------------------------------------------------------------------------------------------------
bool Map::addFrames(int frameID1, int frameID2, Transformation &transform)
{
	// match frame to frame (current with previous)
	if (computeTransformation(
			frameID1,
			frameID2,
			_bufferFrameData1,
			_bufferFrameData2,
			transform))
	{
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

		// free data
		_bufferFrameData1.releaseData();
		// reassign the last frame to avoid reloading all the data twice
		_bufferFrameData1.assignData(_bufferFrameData2);

		return true;
	}
	else
	{
		// empty buffers - data has to be reloaded
		_bufferFrameData1.releaseData();
		_bufferFrameData2.releaseData();

		return false;
	}
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
		int stepFrame = Config::_MatchingRatioFrame;

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
				std::cerr << "\tResync attempt #" <<  Config::_MatchingRatioFrame-stepFrame+1 << "\n";
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
			stepFrame = Config::_MatchingRatioFrame;
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
	
	_sequenceTransform.clear();

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

			//std::cout << transfo._matrix << std::endl;
			if (transfo._idOrig >= minFrameID && (transfo._idDest <= maxFrameID || maxFrameID<0))
				_sequenceTransform.push_back(transfo);

			fileTransfo.ignore(256, '\n');	// to reach end of line
			fileTransfo.peek();				// to update the eof flag (if no empty line)
		}
	}
}


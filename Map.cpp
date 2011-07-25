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
#include "Graph.h"
#include "Map.h"

#include <iostream>
#include <fstream>

#include <vector>

using namespace std;


//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
Graph g_graph;

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
		if (iPose % Config::_RatioFramePCD != 0)
			continue;	// skip frame

		cout << "----------------------------------------------------------------------------\n";
		cout << "Append point cloud frame #" << cameraPoses[iPose]._id << " (" << iPose+1 << "/" << cameraPoses.size() << ")\n";
		cout << cameraPoses[iPose]._matrix << std::endl;

		getFramePointCloud(cameraPoses[iPose]._id, cloudFrame);

		// subsample frame
		subsamplePointCloud(cloudFrame, Config::_RatioKeepSubsamplePCD);
		
		// apply transformation to the point cloud
		pcl::getTransformedPointCloud(
				cloudFrame,
				Eigen::Affine3f(cameraPoses[iPose]._matrix),
				cloudFrameTransformed); 
		
		// apend transformed point cloud
		cloudFull += cloudFrameTransformed;
		cout << " Total Size: " << cloudFull.size() << " points." << std::endl;
		
		if (cloudFull.size() > Config::_MaxNbPointsPCD)
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
		//subsamplePointCloud(cloudFull, Config::_RatioKeepSubsamplePCD);
		
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
	return (getAngleTransform(transformMat) > 10 || getDistanceTransform(transformMat) > 0.5);
}

// -----------------------------------------------------------------------------------------------------
//  loadPoses
// -----------------------------------------------------------------------------------------------------
void loadPoses(PoseVector &poses, const char *filename)
{
	char buf[256];
	sprintf(buf, "%s/%s", Config::_ResultDirectory.c_str(), filename);
	std::ifstream filePoses(buf);
	Pose pose;

	poses.clear();
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

void Map::regeneratePCD()
{
	PoseVector	cameraPoses;

	// initia graph
    g_graph.initialize();

	g_graph.load("graph_initial.g2o");

	// extract the updated camera positions from the optimized graph
	g_graph.extractAllPoses(cameraPoses);

	// build initial point cloud
	if (Config::_GenerateInitialPCD)
		generateMapPCD(cameraPoses, "cloud_initial");

	//  optimized graph
	g_graph.load("graph_optimized.g2o");

	// extract the updated camera positions from the optimized graph
	g_graph.extractAllPoses(cameraPoses);

	// generate optimized point cloud
	if (Config::_GenerateOptimizedPCD)
		generateMapPCD(cameraPoses, "cloud_optimized");
}

// -----------------------------------------------------------------------------------------------------
//  buildMap
// -----------------------------------------------------------------------------------------------------
void buildMap(const TransformationVector &sequenceTransform)
{
	PoseVector	cameraPoses;
	Pose		currentPose;
	TransformationVector sequenceKeyTransform;
	Transformation keyTransform;
	Transformation transform;
	double totalDistance = 0.0;
	FrameData *pFrameDataLC = NULL;
	FrameData frameDataCurrent;
	vector <int>		idCandidateLC;
	vector <FrameData*>	bufferFrameDataLC;
	Transformation bestTransformLC;
	int indexBestLC;
	bool foundLoopClosure = false;
	bool insertLoopClosure = false;

	// save the graph
	char bufLog[256];
	sprintf(bufLog, "%s/%s", Config::_ResultDirectory.c_str(), "loop_closure.log");
	std::ofstream logLC(bufLog);

	// initialize the graph
    g_graph.initialize();
    
    if (sequenceTransform.size()>0)
    {
		// -------------------------------------------------------------------------------------------
		//  origin
		// -------------------------------------------------------------------------------------------
    	currentPose._matrix = Eigen::Matrix4f::Identity();
    	currentPose._id = sequenceTransform[0]._idOrig;
    	cameraPoses.push_back(currentPose);
		// add vertex for the initial pose
		g_graph.addVertex(currentPose);
		keyTransform._matrix = Eigen::Matrix4f::Identity();
		keyTransform._idOrig = sequenceTransform[0]._idOrig;
		// insert as candidate for loop closure
		pFrameDataLC = new FrameData;
		bufferFrameDataLC.push_back(pFrameDataLC);
		idCandidateLC.push_back(currentPose._id);
		
		// -------------------------------------------------------------------------------------------
		//  loop on the global sequence of transformations
		// -------------------------------------------------------------------------------------------
		for (int iTransform=0; iTransform<sequenceTransform.size(); iTransform++)
		{
			// compute new position of the camera, by cumulating the inverse transforms (right side)
			currentPose._matrix = currentPose._matrix * sequenceTransform[iTransform]._matrix.inverse();
			currentPose._id = sequenceTransform[iTransform]._idDest;
			
			// update key transformation by cumulating the transforms (left side)
			keyTransform._matrix = sequenceTransform[iTransform]._matrix * keyTransform._matrix;
			cout << keyTransform._idOrig << "-" << sequenceTransform[iTransform]._idDest << "\r";
			
			if (isKeyTransform(keyTransform._matrix) || iTransform==sequenceTransform.size()-1)
			{
				// -------------------------------------------------------------------------------------------
				//  new keynode: define camera position in the graph
				// -------------------------------------------------------------------------------------------
				totalDistance += getDistanceTransform(keyTransform._matrix);
				cout << "Distance=" << getDistanceTransform(keyTransform._matrix) << "(m)\tAngle=" << getAngleTransform(keyTransform._matrix) << "(deg)\n";

				// add a vertex = current camera pose
				cameraPoses.push_back(currentPose);
				g_graph.addVertex(currentPose);

				// add an edge = new constraint relative to the key transform
				keyTransform._idDest = currentPose._id;
				sequenceKeyTransform.push_back(keyTransform);
				g_graph.addEdge(keyTransform);

				// remap from this position (relative key transform)
				keyTransform._matrix = Eigen::Matrix4f::Identity();
				keyTransform._idOrig = currentPose._id;
			
				// -------------------------------------------------------------------------------------------
				//  loop closure
				// -------------------------------------------------------------------------------------------
				if (totalDistance > Config::_LoopClosureDistance ||
					getAngleTransform(currentPose._matrix) > Config::_LoopClosureAngle)
				{
					insertLoopClosure = true;
					vector<int> currentCandidates;

					// define sample list
					int nbSamples = Config::_LoopClosureWindowSize;
					if (idCandidateLC.size() < Config::_LoopClosureWindowSize)
						nbSamples = idCandidateLC.size();
					if (foundLoopClosure)
						nbSamples = 1;	// candidate already known
					else
					{
						// generate list of indexes where samples will be pickup once
						int sizeLC = idCandidateLC.size()-20;
						if (sizeLC<0)
							sizeLC = 0;
						for (int i=0; i<sizeLC; i++)	// don't take the 20 last frames
							currentCandidates.push_back(i);
						if (nbSamples>currentCandidates.size())
							nbSamples = currentCandidates.size();
					}

					// check loop closure for every sample
					for (int iSample=0; iSample<nbSamples; iSample++)
					{
						int indexLC;
						if (foundLoopClosure && nbSamples==1)
							indexLC = indexBestLC;	// candidate already known
						else
						{
							// pickup a candidate randomly
							int indexRandom = rand() % currentCandidates.size();
							indexLC = currentCandidates[indexRandom];
							// remove this item from the list
							currentCandidates.erase(currentCandidates.begin() + indexRandom);
						}
						cout << "Checking loop closure " << iSample+1 << "/" << nbSamples;
						cout << "\tCandidate frames: " << idCandidateLC[indexLC] << "-" << currentPose._id;
						cout << "\tTotalDistance=" << totalDistance << "(m)\tAngle=" <<  getAngleTransform(currentPose._matrix) << "(deg)\n";

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
								// it may be improved so don't insert it yet
								insertLoopClosure = false;
								// keep the loop closure info
								bestTransformLC = transform;
								indexBestLC = indexLC;
							}
						}
					}
					cout << "----------------------------------------------------------------------------\n";
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
					g_graph.addEdge(bestTransformLC);

					// remove the candidates included in the ID range of the loop closure
					// this supposes the ID are sorted
					cout << "Removing candidates... " ;
					int sizeLC = idCandidateLC.size();
					for (int indexLC=sizeLC; indexLC>=0; indexLC--)
					{
						if (idCandidateLC[indexLC] >= bestTransformLC._idOrig &&
							idCandidateLC[indexLC] <= bestTransformLC._idDest)
						{
							cout << indexLC << ": id=" << idCandidateLC[indexLC] << "\n";
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
					logLC << "Size Candidate List Before: " << sizeLC << " After: " << idCandidateLC.size() << "\n";

					// reset loop closure
					foundLoopClosure = false;
					bestTransformLC._idOrig = -1;
					bestTransformLC._idDest = -1;
					bestTransformLC._ratioInliers = 0;
					bestTransformLC._matrix = Eigen::Matrix4f::Identity();
					totalDistance = 0;
				}

				// new candidate for loop closure
				// add the current pose
				pFrameDataLC = new FrameData;
				bufferFrameDataLC.push_back(pFrameDataLC);
				idCandidateLC.push_back(currentPose._id);

			} // key pose
		}
		if (foundLoopClosure)
		{
			cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
			cout << "Adding edge for Loop Closure";
			cout << " between vertex " << bestTransformLC._idOrig << " and " << bestTransformLC._idDest << "\n";
			cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
			logLC << "Adding edge for Loop Closure";
			logLC << " between vertex " << bestTransformLC._idOrig << " and " << bestTransformLC._idDest << "\n";
			// add new constraint
			g_graph.addEdge(bestTransformLC);
		}

		g_graph.save("graph_initial.g2o");

		// build initial point cloud
		if (Config::_GenerateInitialPCD)
			generateMapPCD(cameraPoses, "cloud_initial");

		// -------------------------------------------------------------------------------------------
		//  optimize the graph
		// -------------------------------------------------------------------------------------------
		g_graph.optimize();
		g_graph.save("graph_optimized.g2o");

		// extract the updated camera positions from the optimized graph
		g_graph.extractAllPoses(cameraPoses);

		// generate optimized point cloud
		if (Config::_GenerateOptimizedPCD)
			generateMapPCD(cameraPoses, "cloud_optimized");

		savePoses(cameraPoses, "poses.dat");

		// free loop closure data
		for (int indexLC=0; indexLC<bufferFrameDataLC.size(); indexLC++)
			delete bufferFrameDataLC[indexLC];
    }
}

// -----------------------------------------------------------------------------------------------------
//  buildFromSequence
// -----------------------------------------------------------------------------------------------------
void Map::buildFromSequence(std::vector<int> &sequenceFramesID)
{
	if (sequenceFramesID.size()>=2)
	{
		FrameData frameData1, frameData2;
		TransformationVector sequenceTransform;
		Transformation transform;
		bool validTransform;
		int indexLastFrame;
		int nbResyncAttempts=0;

		char buf[256];
		sprintf(buf, "%s/transfo.dat", Config::_ResultDirectory.c_str());
		std::ofstream fileTransform(buf);
		// archive sequence
		fileTransform << sequenceFramesID.size() << " ";
		for (int iFrame=0; iFrame<sequenceFramesID.size(); iFrame++)
			fileTransform << sequenceFramesID[iFrame] << " ";
		fileTransform << std::endl;
		
		indexLastFrame = 0;
		for (int iFrame=1; iFrame<sequenceFramesID.size(); iFrame++)
		{
			// match frame to frame (current with previous)
			validTransform = computeTransformation(
					sequenceFramesID[indexLastFrame],
					sequenceFramesID[iFrame],
					frameData1,
					frameData2,
					transform);
			
			if (!validTransform)
			{
				// no valid transform
				std::cerr << "No valid transformation!!";
				std::cerr << "\tRatio=" << transform._ratioInliers*100 << "% ";
				std::cerr << "\tResync attempt #" <<  nbResyncAttempts+1 << "\n";
				if (++nbResyncAttempts < 3)
					continue;	// skip current frame
				else
					break;		// abort
			}
			
			// valid transform
			nbResyncAttempts = 0;
			sequenceTransform.push_back(transform);
			indexLastFrame = iFrame;

			// archive transform
			fileTransform << transform._idOrig << " " << transform._idDest << " ";
			for (int irow=0; irow<4; irow++)
				for (int icol=0; icol<4; icol++)
					fileTransform << transform._matrix(irow,icol) << " ";
			fileTransform << std::endl;
			
			// free data
			frameData1.releaseData();
			// reassign the last frame to avoid reloading all the data twice
			frameData1.assignData(frameData2);
		}
	    
		buildMap(sequenceTransform);
		
		frameData1.releaseData();
		frameData2.releaseData();
	}
	
}

void Map::buildFromArchive()
{
	vector<int> sequenceFramesID;
	TransformationVector resultingTransformations;
	Transformation transfo;
	int nRead, len;
	
	char buf[256];
	sprintf(buf, "%s/transfo.dat", Config::_ResultDirectory.c_str());
	std::ifstream fileTransfo(buf);
	
	// archive sequence
	fileTransfo >> len;
	std::cout << "Sequence length: " << len << std::endl;
	for (int iFrame=0; iFrame<len; iFrame++)
	{
		fileTransfo >> nRead;
		sequenceFramesID.push_back(nRead);
		std::cout << nRead << " ";
	}
	std::cout << std::endl;
	
	while (! fileTransfo.eof())
	{
		// archive transfo
		fileTransfo >> transfo._idOrig >> transfo._idDest;
		//std::cout << "Restore Matrix "<< transfo._idOrig << "-" << transfo._idDest << ":\n";
		for (int irow=0; irow<4; irow++)
			for (int icol=0; icol<4; icol++)
				fileTransfo >> transfo._matrix(irow,icol);
		
		//std::cout << transfo._matrix << std::endl;
		resultingTransformations.push_back(transfo);
		
		fileTransfo.ignore(256, '\n');	// to reach end of line
		fileTransfo.peek();				// to update the eof flag (if no empty line)
	}
	
	if (sequenceFramesID.size()>1 && resultingTransformations.size()>0)
		buildMap(resultingTransformations);
}

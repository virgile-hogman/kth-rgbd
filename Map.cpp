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
	//cloudTemp.points.resize(cloudFull.size()/2);
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
	
	for (int iPose=0; iPose<cameraPoses.size(); iPose++)
	{
		cout << "----------------------------------------------------------------------------\n";
		cout << "Append point cloud frame #" << cameraPoses[iPose]._id << " (" << iPose+1 << "/" << cameraPoses.size() << ")\n";
		cout << cameraPoses[iPose]._matrix << std::endl;

		getFramePointCloud(cameraPoses[iPose]._id, cloudFrame);

		// subsample frame keeping 70%
		subsamplePointCloud(cloudFrame, 70);
		
		// apply transformation to the point cloud
		pcl::getTransformedPointCloud(
				cloudFrame,
				Eigen::Affine3f(cameraPoses[iPose]._matrix),
				cloudFrameTransformed); 
		
		// apend transformed point cloud
		cloudFull += cloudFrameTransformed;
		cout << " Total Size: " << cloudFull.size() << " points." << std::endl;
		
		/*if (iPose % 50 == 0 || iPose==poseTransformations.size()-1)
			subsamplePointCloud(cloudFull,70);*/
	}
	if (cloudFull.size()>0)
	{
		// subsample final point cloud keeping 70%
		//subsamplePointCloud(cloudFull, 70);
		
		cout << "Saving global point cloud binary..." << std::endl;    			
		sprintf(buf_full, "%s/%s", Config::_ResultDirectory.c_str(), filenamePCD);
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
//  buildMap
// -----------------------------------------------------------------------------------------------------
void buildMap(const TransformationVector &sequenceTransform)
{
	TransformationVector sequenceKeyTransform;
	Transformation keyTransform;
	Transformation transform;
	Eigen::Matrix4f currentPoseMat(Eigen::Matrix4f::Identity());	// origin
	Eigen::Matrix4f keyTransformMat(Eigen::Matrix4f::Identity());	// transform from origin
	PoseVector	cameraPoses;
	Pose		cameraPose;
	int nbPose=0;
	FrameData *pLoopClosureFrameData = NULL;
	FrameData frameDataCurrent;
	bool found = false;
	double totalDistance = 0.0;
	
	vector <int>		loopClosureVectorId;
	vector <FrameData*>	loopClosureVectorData;

	Transformation bestTransformLoopClosure;
	float bestRatioLoopClosure = 0;
	int idPoseLoopClosureFrom = -1;
	int idPoseLoopClosureTo = -1;
	bool foundLoopClosure = false;
	bool betterLoopClosure = false;

	char buf[256];
	sprintf(buf, "%s/poses.dat", Config::_ResultDirectory.c_str());
	std::ofstream filePoses(buf);

	// initialize the graph
    g_graph.initialize();
    
    if (sequenceTransform.size()>0)
    {
		// -------------------------------------------------------------------------------------------
		//  origin
		// -------------------------------------------------------------------------------------------
		// add vertex for the initial pose
		g_graph.addVertex(0, currentPoseMat);
		// origin
		cameraPose._matrix = currentPoseMat;
		cameraPose._id = sequenceTransform[0]._idOrig;
		cameraPoses.push_back(cameraPose);
		loopClosureVectorId.push_back(0);
		pLoopClosureFrameData = new FrameData;
		loopClosureVectorData.push_back(pLoopClosureFrameData);
		nbPose++;
		
		keyTransform._idOrig = sequenceTransform[0]._idOrig;
		
		// -------------------------------------------------------------------------------------------
		//  loop on the global sequence of transformations
		// -------------------------------------------------------------------------------------------
		for (int iTransform=0; iTransform<sequenceTransform.size(); iTransform++)
		{
			// compute new position of the camera, by cumulating the inverse transforms (right side)
			currentPoseMat = currentPoseMat * sequenceTransform[iTransform]._matrix.inverse();
			//std::cerr << "Camera position: " << sequenceTransform[iTransform]._idOrig << "-" << sequenceTransform[iTransform]._idDest << "\n" << currentPoseMat << std::endl;
			
			// update key transformation by cumulating the transforms (left side)
			keyTransformMat = sequenceTransform[iTransform]._matrix * keyTransformMat;
			
			cout << keyTransform._idOrig << "-" << sequenceTransform[iTransform]._idDest << "\r";
			if (isKeyTransform(keyTransformMat) || iTransform==sequenceTransform.size()-1)
			{
				// -------------------------------------------------------------------------------------------
				//  new keynode: define camera position in the graph
				// -------------------------------------------------------------------------------------------
				totalDistance += getDistanceTransform(keyTransformMat);
				cout << "Distance=" << getDistanceTransform(keyTransformMat) << "(m)\tAngle=" << getAngleTransform(keyTransformMat) << "(deg)\n";

				keyTransform._idDest = sequenceTransform[iTransform]._idDest;
				keyTransform._matrix = keyTransformMat;

				// add a new vertex at current camera position
				g_graph.addVertex(nbPose, currentPoseMat);
				cameraPose._matrix = currentPoseMat;
				cameraPose._id = sequenceTransform[iTransform]._idDest;
				cameraPoses.push_back(cameraPose);

				// add the edge = new constraint relative to the key transform
				g_graph.addEdge(nbPose-1, nbPose, keyTransform);
				sequenceKeyTransform.push_back(keyTransform);
				nbPose++;

				// remap from this position (relative key transform)
				keyTransformMat = Eigen::Matrix4f::Identity();
				keyTransform._idOrig = sequenceTransform[iTransform]._idDest;
			
				// -------------------------------------------------------------------------------------------
				//  check for loop closure
				// -------------------------------------------------------------------------------------------
				if (totalDistance > Config::_LoopClosureDistance ||
					getAngleTransform(currentPoseMat) > Config::_LoopClosureAngle)
				{
					// looking for a better loop closure
					betterLoopClosure = false;

					for (int iLoopClosure=0; iLoopClosure<loopClosureVectorId.size(); iLoopClosure++)
					{
						cout << "Checking loop closure ";
						cout << "\tCandidate frames: " << sequenceTransform[loopClosureVectorId[iLoopClosure]]._idOrig << "-" << sequenceTransform[iTransform]._idDest;
						cout << "\tTotalDistance=" << totalDistance << "(m)\tAngle=" <<  getAngleTransform(currentPoseMat) << "(deg)\n";

						// loop closure with frame
						bool validTransform = computeTransformation(
								cameraPoses[loopClosureVectorId[iLoopClosure]]._id,
								sequenceTransform[iTransform]._idDest,
								*loopClosureVectorData[iLoopClosure],
								frameDataCurrent,
								transform);

						if (validTransform)
						{
							cout << " LOOP CLOSURE DETECTED (" << sequenceTransform[loopClosureVectorId[iLoopClosure]]._idOrig;
							cout << "-" << sequenceTransform[iTransform]._idDest <<  ") \n";
							cout << "Ratio: " << transform._ratioInliers << "\n";

							if (transform._ratioInliers >= bestRatioLoopClosure)
							{
								// a better loop closure is found
								foundLoopClosure = true;
								// it may be improved so don't insert it yet
								betterLoopClosure = true;
								// keep the loop closure info
								bestTransformLoopClosure = transform;
								bestRatioLoopClosure = transform._ratioInliers;
								idPoseLoopClosureFrom = loopClosureVectorId[iLoopClosure];
								idPoseLoopClosureTo = nbPose-1;
							}
						}
					}
					cout << "----------------------------------------------------------------------------\n";
				}

				// trigger the loop closure unless a better one has just been found
				if (foundLoopClosure && !betterLoopClosure)
				{
					cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
					cout << "Adding edge for Loop Closure";
					cout << " between vertex " << idPoseLoopClosureFrom << " and " << idPoseLoopClosureTo;
					cout << " - ratio: " << bestRatioLoopClosure << "\n";
					cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
					// add the edge = new constraint
					g_graph.addEdge(idPoseLoopClosureFrom, idPoseLoopClosureTo, bestTransformLoopClosure);
					// reset loop closure
					foundLoopClosure = false;
					idPoseLoopClosureFrom = -1;
					idPoseLoopClosureTo = -1;
					bestRatioLoopClosure = 0;
					totalDistance = 0;
				}

				// new candidate for loop closure
				if (loopClosureVectorId.size() < Config::_LoopClosureWindowSize)
				{
					// add the current pose
					loopClosureVectorId.push_back(nbPose-1);
					pLoopClosureFrameData = new FrameData;
					loopClosureVectorData.push_back(pLoopClosureFrameData);
				}

			} // key pose
		}
		if (foundLoopClosure)
		{
			cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
			cout << "Adding edge for Loop Closure";
			cout << " between vertex " << idPoseLoopClosureFrom << " and " << idPoseLoopClosureTo;
			cout << " - ratio: " << bestRatioLoopClosure << "\n";
			cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";
			// add new constraint
			g_graph.addEdge(idPoseLoopClosureFrom, idPoseLoopClosureTo, bestTransformLoopClosure);
		}

		g_graph.save("graph_initial.g2o");

		// build initial point cloud
		if (Config::_GenerateInitialPCD)
			generateMapPCD(cameraPoses, "cloud_initial.pcd");

		// -------------------------------------------------------------------------------------------
		//  optimize the graph
		// -------------------------------------------------------------------------------------------
		g_graph.optimize();
		g_graph.save("graph_optimized.g2o");

		// extract the updated camera position from the optimized graph
		for (int i=0; i<cameraPoses.size(); i++)
		{
			// get the pose from the graph)
			if (! g_graph.getPose(i, cameraPose))
			{
				cerr << "Error in graph extraction!" << std::endl;
				// cut the remaining poses, they are not valid anymore
				cameraPoses.resize(i);
				break;
			}

			// update matrix
			cameraPoses[i]._matrix = cameraPose._matrix;

			filePoses << "Camera [" << i << "] - Frame #" << cameraPoses[i]._id << "\n";
			filePoses << cameraPoses[i]._matrix << "\n----------------------------------------\n";
		}

		// generate optimized point cloud
		if (Config::_GenerateOptimizedPCD)
			generateMapPCD(cameraPoses, "cloud_optimized.pcd");

		// free loop closure data
		for (int i=0; i<loopClosureVectorData.size(); i++)
			delete loopClosureVectorData[i];
    }
}

// -----------------------------------------------------------------------------------------------------
//  buildFromSequence
// -----------------------------------------------------------------------------------------------------
void Map::buildFromSequence(std::vector<int> &sequenceFramesID, bool savePointCloud)
{
	if (sequenceFramesID.size()>=2)
	{
		FrameData frameData1, frameData2;
		TransformationVector sequenceTransform;
		Transformation transform;
		bool validTransform;

		char buf[256];
		sprintf(buf, "%s/transfo.dat", Config::_ResultDirectory.c_str());
		std::ofstream fileTransform(buf);
		// archive sequence
		fileTransform << sequenceFramesID.size() << " ";
		for (int iFrame=0; iFrame<sequenceFramesID.size(); iFrame++)
			fileTransform << sequenceFramesID[iFrame] << " ";
		fileTransform << std::endl;
		
		for (int iFrame=1; iFrame<sequenceFramesID.size(); iFrame++)
		{
			// match frame to frame (current with previous)
			validTransform = computeTransformation(
					sequenceFramesID[iFrame-1],
					sequenceFramesID[iFrame],
					frameData1,
					frameData2,
					transform);
			
			if (!validTransform)
				break;	// no valid transform => abort the sequence
			
			// archive transform
			fileTransform << transform._idOrig << " " << transform._idDest << " ";
			for (int irow=0; irow<4; irow++)
				for (int icol=0; icol<4; icol++)
					fileTransform << transform._matrix(irow,icol) << " ";
			fileTransform << std::endl;
			
			sequenceTransform.push_back(transform);
			
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

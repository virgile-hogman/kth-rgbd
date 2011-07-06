///////////////////
// SIFT EXTRACTION
///////////////////

#include <Eigen/Geometry>

// PCL includes
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/common/transform.h"
//#include "pcl/filters/voxel_grid.h"

#include "Config.h"
#include "CameraDevice.h"
#include "CommonTypes.h"
#include "FeatureMatching.h"
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

bool generatePointCloud(int frameID, pcl::PointCloud<pcl::PointXYZRGB> &pointCloud)
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
	cout << "Size: " << pointCloud.size() << " points after subsampling " << ratioKeep << "%" << std::endl;
}

// -----------------------------------------------------------------------------------------------------
//  saveMapPointCloud
// -----------------------------------------------------------------------------------------------------
void saveMapPointCloud(const TransformationVector &poseTransformations, bool doCumulateTransfo, const char *filenamePCD)
{
	char buf_full[256];
	sprintf(buf_full, "%s/cloud_full.pcd", Config::_ResultDirectory.c_str());
	pcl::PointCloud<pcl::PointXYZRGB> cloudFull;
	pcl::PointCloud<pcl::PointXYZRGB> cloudFrame;
	pcl::PointCloud<pcl::PointXYZRGB> cloudFrameTransformed;
	
	//Eigen::Vector4f cameraPose(0, 0, 0, 1.0); 
	Eigen::Matrix4f globalTransfo = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f inverseTransfo;
	
	if (poseTransformations.size()>0)
	{
		cout << "Initialize point cloud frame #" << poseTransformations[0]._idOrig << " (1/" << poseTransformations.size()+1 << ")..." << std::endl;
		//char buf[256];
		//sprintf(buf, "%s/cloud%d.pcd", Config::_DataDirectory.c_str(), framesID[0]);
		//if (pcl::io::loadPCDFile(buf, cloudFull) != 0)
		generatePointCloud(poseTransformations[0]._idOrig, cloudFull);
	}
	
	for (int iPose=0; iPose<poseTransformations.size(); iPose++)
	{
		cout << "----------------------------------------------------------------------------" << std::endl;
		pcl::PointXYZRGB ptCameraPose;

		// compute global transformation from the start
		if (doCumulateTransfo)
			globalTransfo = poseTransformations[iPose]._matrix * globalTransfo;
		else
			globalTransfo = poseTransformations[iPose]._matrix;
			
		//cout << globalTransfo.inverse() << std::endl;
		//cout << "Mean error:" << poseTransformations[iPose]._error << std::endl;
		// update global point cloud

		cout << "Generating point cloud frame #" << poseTransformations[iPose]._idDest << " (" << iPose+2 << "/" << poseTransformations.size()+1 << ")...";
		//char buf[256];
		//sprintf(buf, "%s/cloud%d.pcd", Config::_DataDirectory.c_str(), framesID[iPose+1]);
		//if (pcl::io::loadPCDFile(buf, cloudFrame) != 0)
		generatePointCloud(poseTransformations[iPose]._idDest, cloudFrame);

		// subsample frame keeping 70%
		subsamplePointCloud(cloudFrame, 70);
		
		// inverse transform
		inverseTransfo = globalTransfo.inverse();
		
		// apply transformation to the point cloud
		pcl::getTransformedPointCloud(
				cloudFrame,
				Eigen::Affine3f(inverseTransfo),
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


// -----------------------------------------------------------------------------------------------------
// saveMaps
// -----------------------------------------------------------------------------------------------------
void saveMaps(const TransformationVector &resultingTransformations)
{
	TransformationVector transfosFromGraph;
	
	g_graph.setSaveDirectory(Config::_ResultDirectory.c_str());
	g_graph.save("graph_initial.g2o");
	
	// build initial map and point cloud, cumulate the transfos
	saveMapPointCloud(resultingTransformations, true, "cloud_initial.pcd");
	
	// optimize the graph
	g_graph.optimize();
	g_graph.save("graph_optimized.g2o");
	
	// extract the transformations from the optimized graph
	for (int i=0; i<resultingTransformations.size()-1; i++)
	{		
		Transformation tfoGraph;
		bool valid = g_graph.getTransfo(i+1, tfoGraph);
		if (! valid)
			break;
				
		tfoGraph._idOrig = resultingTransformations[i]._idOrig;
		tfoGraph._idDest = resultingTransformations[i]._idDest;
		transfosFromGraph.push_back(tfoGraph);
		//cout << "camera pose graph\n" << cumulatedTransformationGraph.inverse() << std::endl;
	}
	// build optimized map and point cloud, don't cumulate the transfos (already done in the graph)
	saveMapPointCloud(transfosFromGraph, false, "cloud_optimized.pcd");
}


// -----------------------------------------------------------------------------------------------------
//  buildMap
// -----------------------------------------------------------------------------------------------------
void buildMap(const TransformationVector &resultingTransformations)
{
	TransformationVector keyPoseTransformations;
	Transformation transfo;
	Transformation keyTransfo;
	Eigen::Matrix4f cameraPoseMat(Eigen::Matrix4f::Identity());
	Eigen::Matrix4f keyPoseMat(Eigen::Matrix4f::Identity());
	int nbPose=0;
	
	FrameData frameDataLoopClosure;
	FrameData frameDataCurrent;
	bool found = false;
	
	// initialize the graph
    g_graph.initialize();
    
    if (resultingTransformations.size()>0)
    {
		// add vertex for the initial pose
		g_graph.addVertex(0, cameraPoseMat);
		nbPose++;
		
		keyTransfo._idOrig = resultingTransformations[0]._idOrig;
		
		for (int i=0; i<resultingTransformations.size(); i++)
		{
			transfo = resultingTransformations[i];
			
			cameraPoseMat = cameraPoseMat * transfo._matrix.inverse();
			std::cerr << "Camera mat " << transfo._idOrig << "-" << transfo._idDest << "\n" << cameraPoseMat << std::endl;
			
			keyPoseMat = keyPoseMat * transfo._matrix.inverse();
			
			if (i%5==0 || i==resultingTransformations.size()-1)	// TODO - define better criterion for keynode
			{
				// new keynode
				keyTransfo._idDest = resultingTransformations[i]._idDest;
				keyTransfo._matrix = keyPoseMat.inverse();

				// add a new vertex
				g_graph.addVertex(nbPose, cameraPoseMat);
		
				// add the edge = constraints
				g_graph.addEdge(nbPose-1, nbPose, keyTransfo);
				
				keyPoseTransformations.push_back(keyTransfo);
				nbPose++;

				// remap from the last keynode
				keyPoseMat = Eigen::Matrix4f::Identity();
				keyTransfo._idOrig = resultingTransformations[i]._idDest;
			}
			
			if (i>20 && i%5 == 0 && !found)
			{
				// loop closure with frame #1
				bool validMatch = matchFrames(
						resultingTransformations[0]._idOrig,
						resultingTransformations[i]._idDest,
						frameDataLoopClosure,
						frameDataCurrent,
						true,
						transfo);
				
				if (validMatch)
				{
				    // add the edge = constraints
					std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
					std::cerr << " LOOP CLOSURE DETECTED " << std::endl;
					std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
				    g_graph.addEdge(0, nbPose-1, transfo);
				    found = true;
				}
			}
			
		}
		
		saveMaps(keyPoseTransformations);
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
		TransformationVector resultingTransformations;
		Transformation transfo;
		bool validMatch;

		char buf[256];
		sprintf(buf, "%s/transfo.dat", Config::_ResultDirectory.c_str());
		std::ofstream fileTransfo(buf);
		// archive sequence
		fileTransfo << sequenceFramesID.size() << " ";
		for (int iFrame=0; iFrame<sequenceFramesID.size(); iFrame++)
			fileTransfo << sequenceFramesID[iFrame] << " ";
		fileTransfo << std::endl;
		
		for (int iFrame=1; iFrame<sequenceFramesID.size(); iFrame++)
		{
			// match frame to frame (current with previous)
			validMatch = matchFrames(
					sequenceFramesID[iFrame-1],
					sequenceFramesID[iFrame],
					frameData1,
					frameData2,
					true,
					transfo);
			
			if (!validMatch)
				break;	// no valid transform
			
			// archive transfo
			fileTransfo << transfo._idOrig << " " << transfo._idDest << " ";
			for (int irow=0; irow<4; irow++)
				for (int icol=0; icol<4; icol++)
					fileTransfo << transfo._matrix(irow,icol) << " ";
			fileTransfo << std::endl;
			
			resultingTransformations.push_back(transfo);
			
			// free data
			frameData1.releaseData();
			// reassign the last frame to avoid reloading all the data twice
			frameData1.assignData(frameData2);
		}
	    
		buildMap(resultingTransformations);
		
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
		std::cout << "Restore Matrix "<< transfo._idOrig << "-" << transfo._idDest << ":\n";
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

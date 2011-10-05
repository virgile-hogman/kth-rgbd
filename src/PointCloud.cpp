#include "FrameData.h"
#include "CameraDevice.h"
#include "Config.h"
#include "PointCloud.h"

// PCL includes
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/common/transform.h"
#include "pcl/registration/icp.h"

using namespace std;

bool getFramePointCloud(int frameID, pcl::PointCloud<pcl::PointXYZ> &pointCloud)
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
			if (depth != 0)
			{
				pcl::PointXYZ pt;

				// locate point in meters
				pt.z = (ind_x - NBPIXELS_X_HALF) * depth * constant;
				pt.y = (NBPIXELS_Y_HALF - ind_y) * depth * constant;
				pt.x = depth * 0.001 ; // given depth values are in mm

				pointCloud.push_back(pt);
			}
		}
	}

	return true;
}

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
			if (depth != 0)
			{
				pcl::PointXYZRGB pt;

				// locate point in meters
				pt.z = (ind_x - NBPIXELS_X_HALF) * depth * constant;
				pt.y = (NBPIXELS_Y_HALF - ind_y) * depth * constant;
				pt.x = depth * 0.001 ; // given depth values are in mm

				// reinterpret color bytes
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
//  getTransformICP
// -----------------------------------------------------------------------------------------------------
bool PointCloud::getTransformICP(const FrameData &frameData1, const FrameData &frameData2, Eigen::Matrix4f& transformation)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> cloudFinal;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	// load the point clouds
	// TODO: optimize method, very low performance as the RGB+D data is reloaded (just for ICP test)
	// TODO: optimize memory management by keeping at least the last frame
	// we want to align the frame2 (source) on the frame1 (target)
	getFramePointCloud(frameData1.getFrameID(), *cloudTarget);
	getFramePointCloud(frameData2.getFrameID(), *cloudSource);

	// define the inputs
	icp.setInputCloud(cloudSource);
	icp.setInputTarget(cloudTarget);

	icp.setRANSACOutlierRejectionThreshold(Config::_MatchingMaxDistanceInlier);
	/* use default values
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (0.05);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (50);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (1);
	*/

	std::cout << "Running ICP...";
	fflush(stdout);

	// run the ICP loop
	icp.align(cloudFinal, transformation);

	std::cout << " has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	if (icp.hasConverged())
		transformation = icp.getFinalTransformation();

	return icp.hasConverged();
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
//  generatePCD
// -----------------------------------------------------------------------------------------------------
void PointCloud::generatePCD(const PoseVector &cameraPoses, const char *filenamePCD)
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

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

#include "FrameData.h"
#include "CameraDevice.h"
#include "Config.h"
#include "PointCloud.h"

// PCL includes
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/common/transforms.h"
#include "pcl/registration/icp.h"

using namespace std;


bool getFramePointCloud(int frameID, pcl::PointCloud<pcl::PointXYZRGB> &pointCloud)
{
	FrameData frameData;
	if (!frameData.loadImageRGBD(frameID)) {
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

	float focalInv = 0.001 / Config::_FocalLength;
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
				pt.z = (ind_x - NBPIXELS_X_HALF) * depth * focalInv;
				pt.y = (NBPIXELS_Y_HALF - ind_y) * depth * focalInv;
				pt.x = depth * 0.001 ; // depth values are given in mm

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
bool PointCloud::getTransformICP(
		const FrameData &frameData1,
		const FrameData &frameData2,
		const vector<Eigen::Vector3f> &source,
		const vector<Eigen::Vector3f> &target,
		Eigen::Matrix4f& transformation)
{
	//TEMPORARILY DISABLED ICP PCL FOR CONFLICT FLANN OPENCV
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSource (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTarget (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> cloudFinal;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	pcl::PointXYZ pt;
	for (int i=0; i<source.size(); i++) {
		pt.x = source[i][0];
		pt.y = source[i][1];
		pt.z = source[i][2];
		cloudSource->push_back(pt);
	}
	for (int i=0; i<target.size(); i++) {
		pt.x = target[i][0];
		pt.y = target[i][1];
		pt.z = target[i][2];
		cloudTarget->push_back(pt);
	}

	// define the inputs
	// we invert here in the sense we want the transfo from frameTarget to frameSource
	icp.setInputCloud(cloudSource);
	icp.setInputTarget(cloudTarget);

	icp.setRANSACOutlierRejectionThreshold(Config::_MatchingMaxDistanceInlier);
	// use default values
	// // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	// icp.setMaxCorrespondenceDistance (0.05);
	// // Set the maximum number of iterations (criterion 1)
	// icp.setMaximumIterations (50);
	// // Set the transformation epsilon (criterion 2)
	// icp.setTransformationEpsilon (1e-8);
	// // Set the euclidean distance difference epsilon (criterion 3)
	// // icp.setEuclideanFitnessEpsilon (1);

	std::cout << "Running ICP...";
	fflush(stdout);

	// run the ICP loop
	icp.align(cloudFinal, transformation);

	std::cout << " has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	if (icp.hasConverged())
		transformation = icp.getFinalTransformation();

	return icp.hasConverged();
	//TEMPORARILY DISABLED ICP PCL FOR CONFLICT FLANN OPENCV
	return false;
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
		if (iPose>0 && iPose%Config::_PcdRatioFrame != 0)
			continue;	// skip frame

		cout << "----------------------------------------------------------------------------\n";
		cout << "Append point cloud frame=#" << cameraPoses[iPose]._id;
		cout << " pose=" << iPose+1 << "/" << cameraPoses.size();
		cout << " ratio_pcd=1/" << Config::_PcdRatioFrame;
		cout << " cloud=" << (iPose/Config::_PcdRatioFrame)+1 << "/" << ((cameraPoses.size()-1)/Config::_PcdRatioFrame)+1 << "\n";
		cout << cameraPoses[iPose]._matrix << std::endl;

		getFramePointCloud(cameraPoses[iPose]._id, cloudFrame);

		// subsample frame
		subsamplePointCloud(cloudFrame, Config::_PcdRatioKeepSubsample);

		// apply transformation to the point cloud
		pcl::transformPointCloud(
				cloudFrame,
				cloudFrameTransformed,
				Eigen::Affine3f(cameraPoses[iPose]._matrix));

		// apend transformed point cloud
		cloudFull += cloudFrameTransformed;
		cout << " Total Size: " << cloudFull.size() << " points (";
		cout <<  cloudFull.size()*100/Config::_PcdMaxNbPoints << "% of max capacity)." << std::endl;

		if (cloudFull.size() > Config::_PcdMaxNbPoints)
		{
			// max size reached
			cout << "Saving global point cloud binary...\n";
			sprintf(buf_full, "%s/%s_%02d.pcd", Config::_PathDataProd.c_str(), filenamePCD, nbPCD++);
			cout << "File: " << buf_full << "\n";
			pcl::io::savePCDFile(buf_full, cloudFull, true);
			// bug in PCL - the binary file is not created with the good permissions!
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
			sprintf(buf_full, "%s/%s_%02d.pcd", Config::_PathDataProd.c_str(), filenamePCD, nbPCD);
		else
			sprintf(buf_full, "%s/%s.pcd", Config::_PathDataProd.c_str(), filenamePCD);
		cout << "File: " << buf_full << "\n";
		pcl::io::savePCDFile(buf_full, cloudFull, true);
		// bug in PCL - the binary file is not created with the good permissions!
		char bufsys[256];
		sprintf(bufsys, "chmod a+rw %s", buf_full);
		system(bufsys);
	}
}

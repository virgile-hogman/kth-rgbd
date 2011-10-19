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

// PCL includes
#include "pcl/common/transformation_from_correspondences.h"

// Open CV
#include "opencv/cv.h"
#include "opencv/highgui.h"

extern "C" {
#include "sift.h"
#include "imgfeatures.h"
#include "kdtree.h"
#include "utils.h"	// stack_imgs
#include "xform.h"
}

#include "CameraDevice.h"
#include "Config.h"
#include "CommonTypes.h"
#include "FrameData.h"
#include "Matching.h"
#include "TimeTracker.h"
#include "PointCloud.h"

#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS		200

// -----------------------------------------------------------------------------------------------------
//  evaluateTransform
// -----------------------------------------------------------------------------------------------------
void evaluateTransform(
		const Eigen::Matrix4f& transformation,
        const std::vector<Eigen::Vector3f> &matchesSource,
        const std::vector<Eigen::Vector3f> &matchesTarget,
        double maxError,        
        std::vector<int> &inliers,
        double &meanError,
        float &ratio)
{
	inliers.clear();
	meanError = 0.0;
	ratio = 0.0;

	// for every matching point
	for (unsigned int id = 0; id < matchesSource.size(); id++)
	{
		// vectors with homogeneous coordinates
		Eigen::Vector4f source(matchesSource[id][0], matchesSource[id][1], matchesSource[id][2], 1.0);
		Eigen::Vector4f target(matchesTarget[id][0], matchesTarget[id][1], matchesTarget[id][2], 1.0);

		// project the original point and compute the difference vector wrt the match
		Eigen::Vector4f vectorDiff = (transformation * source) - target;

		// compute the error
		double error = vectorDiff.squaredNorm();

		// check and ignore outlier
		if (error > maxError)
			continue;

		// keep the inlier
		inliers.push_back(id);
		meanError += sqrt(error);
	}

	if (inliers.size()>0)
		meanError /= inliers.size();
	else
		meanError = -1.0;

	ratio = (float)inliers.size()/matchesSource.size();
}

// -----------------------------------------------------------------------------------------------------
//  drawInliers
// -----------------------------------------------------------------------------------------------------
void drawInliers(
		FrameData &frameData1,
		FrameData &frameData2,
		const vector<int> &indexMatches,
		const vector<int> &indexBestInliers,
		const vector<int> &initialPairs,
		bool forLoopClosure)
{
	IplImage* imgStackedInliers = NULL;
	CvPoint pt1, pt2;
	CvFont font;
	double hScale=0.5;
	double vScale=0.5;
	int    lineWidth=1;
	// define a font to write some text
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale,vScale, 0, lineWidth);

	if (frameData1.getImage() != NULL && frameData2.getImage() != NULL) {
		// stack the 2 images
		imgStackedInliers = stack_imgs( frameData1.getImage(), frameData2.getImage() );

		// draw red lines for outliers
		// all the initial matches are drawn here - the inliers will be overwritten with green
		for (int i=0; i<indexMatches.size(); i++)
		{
			int idMatch = indexMatches[i];
			const struct feature* feat1 = frameData1.getFeature(idMatch);
			const struct feature* feat2 = frameData1.getFeatureMatch(idMatch);

			// draw a line through the 2 points in the stacked image
			pt1 = cvPoint( cvRound( feat1->x ), cvRound( feat1->y ) );
			pt2 = cvPoint( cvRound( feat2->x ), cvRound( feat2->y ) );
			pt2.y += frameData1.getImage()->height;
			// draw a green line
			cvLine( imgStackedInliers, pt1, pt2, CV_RGB(255,0,0), 1, 8, 0 );
		}
		// draw green lines for the best inliers
		for (int i=0; i<indexBestInliers.size(); i++)
		{
			int idInlier = indexBestInliers[i];
			int idMatch = indexMatches[idInlier];
			const struct feature* feat1 = frameData1.getFeature(idMatch);
			const struct feature* feat2 = frameData1.getFeatureMatch(idMatch);

			// draw a line through the 2 points in the stacked image
			pt1 = cvPoint( cvRound( feat1->x ), cvRound( feat1->y ) );
			pt2 = cvPoint( cvRound( feat2->x ), cvRound( feat2->y ) );
			pt2.y += frameData1.getImage()->height;
			// draw a green line
			cvLine( imgStackedInliers, pt1, pt2, CV_RGB(0,255,0), 1, 8, 0 );
		}
		// draw lines for the 3 initial points
		for (int i=0; i<initialPairs.size(); i++)
		{
			int idMatch = initialPairs[i];
			const struct feature* feat1 = frameData1.getFeature(idMatch);
			const struct feature* feat2 = frameData1.getFeatureMatch(idMatch);

			// draw a line through the 2 points in the stacked image
			pt1 = cvPoint( cvRound( feat1->x ), cvRound( feat1->y ) );
			pt2 = cvPoint( cvRound( feat2->x ), cvRound( feat2->y ) );
			pt2.y += frameData1.getImage()->height;
			// draw a green line
			cvLine( imgStackedInliers, pt1, pt2, CV_RGB(0,0,140), 2, 8, 0 );
		}

		// save stacked image
		char buf[256];
		sprintf(buf,"inliers:%d/%d (%d%%)", indexBestInliers.size(), indexMatches.size(), indexBestInliers.size()*100/indexMatches.size());
		cvPutText(imgStackedInliers, buf, cvPoint(5, 950), &font, cvScalar(255,255,0));
		if (forLoopClosure)
			sprintf(buf, "%s/loopc_%d_%d_inliers.bmp", Config::_ResultDirectory.c_str(), frameData1.getFrameID(), frameData2.getFrameID());
		else
			sprintf(buf, "%s/matching_%d_%d_inliers.bmp", Config::_ResultDirectory.c_str(), frameData1.getFrameID(), frameData2.getFrameID());
		cvSaveImage(buf, imgStackedInliers);
		cvReleaseImage(&imgStackedInliers);
	}
}

// -----------------------------------------------------------------------------------------------------
//  findTransformRANSAC
// -----------------------------------------------------------------------------------------------------
bool findTransformRANSAC(
		FrameData &frameData1,
		FrameData &frameData2,
		vector<int> &indexMatches,
		vector<Eigen::Vector3f>	&matchesSource,
		vector<Eigen::Vector3f>	&matchesTarget,
		Transformation &resultTransform,
		bool forLoopClosure)
{
	bool validTransformation = false;
	// find transform pairs
	pcl::TransformationFromCorrespondences tfc;
	Eigen::Matrix4f bestTransformationMat;
	std::vector<int> indexBestInliers;
	vector<int>	initialPairs;	// to track the 3 first points
	int nbValidMatches = indexMatches.size();
	double bestError = 1E10;	// large value
	float bestRatio = 0;

	int k = 3;	// minimum number of points in a sample
	// int k = nbValidMatches/10;	// minimum number of points in a sample

	if (nbValidMatches < k)
		return false;

	for (int iteration=0; iteration<Config::_MatchingNbIterations; iteration++)
	{
		//printf("\nIteration %d ... \t", iteration+1);
		tfc.reset();
		// pickup k points from matches
		for (int i=0; i<k; i++)
		{
			int id_match = rand() % nbValidMatches;
			tfc.add(matchesSource[id_match], matchesTarget[id_match]);
		}

		// compute transformation from matches
		Eigen::Matrix4f transformation = tfc.getTransformation().matrix();

		// compute error and keep only inliers
		std::vector<int> indexInliers;
		double maxInlierDistance = Config::_MatchingMaxDistanceInlier;
		double meanError;
		float ratio;

		evaluateTransform(transformation,
			matchesSource,
			matchesTarget,
			maxInlierDistance * maxInlierDistance,
			indexInliers,
			meanError,
			ratio);

		//printf("Found %d inliers (%d%%)\tMean error:%f\n", indexInliers.size(), indexInliers.size()*100/nbValidMatches, meanError);

		if (meanError<0 || meanError >= maxInlierDistance)
			continue;	// skip this set of sample points and go for a new iteration

		if (meanError < bestError)
		{
			if (ratio > bestRatio)
				bestRatio = ratio;

			if (indexInliers.size()<Config::_MatchingMinNbInlier || ratio<Config::_MatchingMinRatioInlier)
				continue;	// not enough inliers found
		}

		// ------------------------------------------------
		// recompute a new transformation from the inliers
		// ------------------------------------------------
		//printf("\nRecomputing transfo... \t");
		tfc.reset();
		for (int idInlier = 0; idInlier < indexInliers.size(); idInlier++) {
			int idMatch  = indexInliers[idInlier];
			tfc.add(matchesSource[idMatch], matchesTarget[idMatch]);
		}
		// compute transformation from inliers
		transformation = tfc.getTransformation().matrix();

		evaluateTransform(transformation,
				matchesSource,
				matchesTarget,
				maxInlierDistance * maxInlierDistance,
				indexInliers,
				meanError,
				ratio);

		if (meanError<0 || meanError >= maxInlierDistance)
			continue;	// skip these 3 points and go for a new iteration

		if (meanError < bestError)
		{
			if (ratio > bestRatio)
				bestRatio = ratio;

			if (indexInliers.size()<Config::_MatchingMinNbInlier || ratio<Config::_MatchingMinRatioInlier)
				continue;	// not enough inliers found

			//printf("\t => Best transformation! ", indexInliers.size(), meanError);
			bestTransformationMat = transformation;
			bestError = meanError;
			indexBestInliers = indexInliers;
		}
	}

	if (indexBestInliers.size()>0)
	{
		// RANSAC success
		std::cout << "Best Transformation --->\t" << indexBestInliers.size() << "/" << nbValidMatches;
		std::cout << " inliers (" << indexBestInliers.size()*100/nbValidMatches <<  "%)";
		std::cout << "\terror="<< bestError << std::endl;
		//std::cout << bestTransformationMat << std::endl;

		// ------------------------------------------------
		// recompute (yes, again!) the final transformation from all the best inliers
		// ------------------------------------------------
		tfc.reset();
		for (int i=0; i<indexBestInliers.size(); i++)
		{
			int id_match = indexBestInliers[i];
			tfc.add(matchesSource[id_match], matchesTarget[id_match]);
		}
		// extract transformation
		Eigen::Matrix4f transformation = tfc.getTransformation().matrix();

		// ------------------------------------------------
		// some stats
		// ------------------------------------------------
		// compute mean vector
		Eigen::Vector3f meanVector(0,0,0);
		for (int i=0; i<indexBestInliers.size(); i++)
		{
			int id = indexBestInliers[i];
			meanVector[0]+=matchesTarget[id][0];
			meanVector[1]+=matchesTarget[id][1];
			meanVector[2]+=matchesTarget[id][2];
		}
		meanVector[0]/=indexBestInliers.size();
		meanVector[1]/=indexBestInliers.size();
		meanVector[2]/=indexBestInliers.size();
		// compute variances
		float variance3d=0;
		float variance2d=0;
		for (int i=0; i<indexBestInliers.size(); i++)
		{
			int id = indexBestInliers[i];
			Eigen::Vector3f diff = meanVector - Eigen::Vector3f(matchesTarget[id][0],matchesTarget[id][1],matchesTarget[id][2]);
			variance3d += diff.squaredNorm();
			// set x diff to zero to ignore depth
			diff[0] = 0;
			variance2d += diff.squaredNorm();
		}
		variance3d/=indexBestInliers.size();
		variance2d/=indexBestInliers.size();

		// stats
		char buf[256];
		sprintf(buf, "%s/stats.log", Config::_ResultDirectory.c_str());
		std::ofstream fileStats(buf, ios_base::app);
		fileStats << frameData1.getFrameID() << "-" << frameData2.getFrameID();
		fileStats << "\t" << indexBestInliers.size() << "\t" << nbValidMatches;
		fileStats << "\t" << indexBestInliers.size()*100/nbValidMatches;
		fileStats << "\t" << variance2d << "\t" << variance3d << "\n"  ;

		validTransformation = true;
		resultTransform._matrix = bestTransformationMat;
		resultTransform._error = bestError;
		resultTransform._ratioInliers = float(indexBestInliers.size())/nbValidMatches;

		drawInliers(frameData1, frameData2, indexMatches, indexBestInliers, initialPairs, forLoopClosure);
	}
	else
	{
		// no valid transformation found
		validTransformation = false;
		resultTransform._ratioInliers = bestRatio;
	}

	// compute score for evaluation of quality, relatively to minimum ratio of inliers required
	resultTransform._qualityScore = (resultTransform._ratioInliers-Config::_MatchingMinRatioInlier)
			/(1-Config::_MatchingMinRatioInlier);

	return validTransformation;
}

// -----------------------------------------------------------------------------------------------------
//  kdSearchFeatureMatches
// -----------------------------------------------------------------------------------------------------
void kdSearchFeatureMatches(
		FrameData &frameData1,
		FrameData &frameData2,
		vector<int> &indexMatches,
		vector<Eigen::Vector3f>	&matchesSource,
		vector<Eigen::Vector3f>	&matchesTarget,
		bool forLoopClosure)
{
	TimeTracker tm;
	struct feature** neighbourFeatures = NULL;	// SIFT feature
	struct feature* feat;
	struct kd_node* kdRoot;
	CvPoint pt1, pt2;
	double d0, d1;
	int k, i, nbInitialMatches = 0, nbValidMatches = 0;
	char buf[256];
	float constant = 0.001 / CameraDevice::_FocalLength;    // TODO - redefine this properly
	IplImage* imgStacked = NULL;

	CvFont font;
	double hScale=0.5;
	double vScale=0.5;
	int    lineWidth=1;
	// define a font to write some text
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale,vScale, 0, lineWidth);

	//vector<int>	indexArea1, indexArea2, indexArea3;

	indexMatches.clear();
	matchesSource.clear();
	matchesTarget.clear();

	if (Config::_SaveImageInitialPairs &&
		(frameData1.getImage() != NULL && frameData2.getImage() != NULL)) {
		// stack the 2 images
		imgStacked = stack_imgs(frameData1.getImage(), frameData2.getImage());
	}

	tm.start();
	printf("Frames %03d-%03d:\t Searching for matches... ", frameData1.getFrameID(), frameData2.getFrameID());
	fflush(stdout);

	// try match the new features found in the 2d frame...
	kdRoot = kdtree_build(frameData2.getFeatures(), frameData2.getNbFeatures());
	fflush(stdout);
	for (i=0; i < frameData1.getNbFeatures(); i++)
	{
		// ... looking in the 1st frame
		feat = frameData1.getFeatures() + i;
		// search for 2 nearest neighbours
		fflush(stdout);
		k = kdtree_bbf_knn(kdRoot, feat, 2, &neighbourFeatures, KDTREE_BBF_MAX_NN_CHKS);
		if (k == 2)
		{
			// the neighbours are ordered in increasing descriptor distance
			d0 = descr_dist_sq( feat, neighbourFeatures[0] );
			d1 = descr_dist_sq( feat, neighbourFeatures[1] );
			// the 2d neighbour should be relatively close (robustness check - see D.Lowe paper)
			if (d0 < d1 * Config::_MatchingDistanceRatioNN)
			{
				const struct feature* feat1 = frameData1.getFeature(i);
				const struct feature* feat2 = neighbourFeatures[0];
				// read depth info
				const TDepthPixel depth1 = frameData1.getFeatureDepth(feat1);
				const TDepthPixel depth2 = frameData2.getFeatureDepth(feat2);

				bool bValidMatch = false;

				nbInitialMatches++;
				//printf("Feature %d : ratio2N=%f d0=%f d1=%f x2=%f y2=%f\n", i, d0/d1, d0, d1, feat2->x, feat2->y);

				// draw a line through the 2 points in the stacked image
				if (frameData1.getImage() != NULL) {
					pt1 = cvPoint( cvRound( feat1->x ), cvRound( feat1->y ) );
					pt2 = cvPoint( cvRound( feat2->x ), cvRound( feat2->y ) );
					pt2.y += frameData1.getImage()->height;
				}
				
				// check if depth values are valid
				if (depth1>0 && depth2>0) {
					// convert pixels to metric
					float z1 = (feat1->x - NBPIXELS_X_HALF) * depth1 * constant;
					float y1 = (NBPIXELS_Y_HALF - feat1->y) * depth1 * constant;
					float x1 = depth1 * 0.001 ; // given depth values are in mm
					
					float z2 = (feat2->x - NBPIXELS_X_HALF) * depth2 * constant;
					float y2 = (NBPIXELS_Y_HALF - feat2->y) * depth2 * constant;
					float x2 = depth2 * 0.001 ; // given depth values are in mm

					Eigen::Vector3f orig(x1,y1,z1);
					Eigen::Vector3f dest(x2,y2,z2);
					
					// compute the distance
					Eigen::Vector3f vectorDiff = dest-orig;
					double distance = vectorDiff.squaredNorm();
					
					if (distance<Config::_MatchingMaxDistanceKeypoint) {
						// draw a green line
						if (imgStacked != NULL)
							cvLine( imgStacked, pt1, pt2, CV_RGB(0,255,0), 1, 8, 0 );
						// this is a valid match
						nbValidMatches++;
						// fwd link the previous features to the new features according to the match
						frameData1.setFeatureMatch(i, neighbourFeatures[0]);
						indexMatches.push_back(i);

						matchesSource.push_back(orig);
						matchesTarget.push_back(dest);

						bValidMatch = true;
					}

				}

				// ignore the pairs without depth any info, but show the remaining outliers
				if (!bValidMatch && (depth1>0 || depth2>0)) {
					if (imgStacked != NULL)
					{
						// draw a red line
						cvLine(imgStacked, pt1, pt2, CV_RGB(255,0,0), 1, 8, 0);
						//cvPutText(imgStacked, bufDiff, cvPoint((pt1.x+pt2.x)/2 -20,(pt1.y+pt2.y)/2), &font, cvScalar(255,255,0));
						sprintf(buf,"_%u", depth1);
						cvPutText(imgStacked, buf, cvPoint(pt1.x, pt1.y), &font, cvScalar(255,255,0));
						sprintf(buf,"_%u", depth2);
						cvPutText(imgStacked, buf, cvPoint(pt2.x, pt2.y), &font, cvScalar(255,255,0));
					}
				}
			}
		}
		if (neighbourFeatures != NULL)
			free(neighbourFeatures);
	}
	fflush(stdout);
	
	// free memory
	kdtree_release(kdRoot);
	
	int ratio=0;
	if (nbInitialMatches!=0)
		ratio = nbValidMatches*100/nbInitialMatches;
	sprintf(buf,"Matches:%d/%d (%d%%)", nbValidMatches, nbInitialMatches, ratio);

	if (imgStacked != NULL)
	{
		cvPutText(imgStacked, buf, cvPoint(5, 950), &font, cvScalar(255,255,0));
		if (forLoopClosure)
			sprintf(buf, "%s/loopc_%d_%d.bmp", Config::_ResultDirectory.c_str(), frameData1.getFrameID(), frameData2.getFrameID());
		else
			sprintf(buf, "%s/matching_%d_%d.bmp", Config::_ResultDirectory.c_str(), frameData1.getFrameID(), frameData2.getFrameID());

		// save stacked image
		cvSaveImage(buf, imgStacked);
		cvReleaseImage(&imgStacked);
	}

	tm.stop();
	printf("\tMatches: %d/%d (%d%%).\t(%dms)\n", nbValidMatches, nbInitialMatches, ratio, tm.duration() );
	fflush(stdout);
}

// -----------------------------------------------------------------------------------------------------
//  computeTransformation
// -----------------------------------------------------------------------------------------------------
bool computeTransformation(
		int frameID1,
		int frameID2,
		FrameData &frameData1,
		FrameData &frameData2,
		Transformation &resultingTransform,
		bool forLoopClosure)
{
	TimeTracker tm;

	vector<int> indexMatches;
	vector<Eigen::Vector3f>	matchesSource;
	vector<Eigen::Vector3f>	matchesTarget;
	bool validTransform = false;

	int nbValidMatches = indexMatches.size();

	resultingTransform._matrix = Eigen::Matrix4f::Identity();
	resultingTransform._error = 1.0;
	resultingTransform._idOrig = frameID1;
	resultingTransform._idDest = frameID2;
	resultingTransform._ratioInliers = 0;

	// ---------------------------------------------------------------------------
	// feature extraction
	// ---------------------------------------------------------------------------
	tm.start();
	printf("Frames %03d-%03d:\t Extracting %s features... ", frameID1, frameID2,
			Config::_FeatureType==0?"SIFT":"SURF");
	fflush(stdout);

	// load data Frame1
	if (! frameData1.fetchFeatures(frameID1))
		return false;

	// load data Frame2
	if (! frameData2.fetchFeatures(frameID2))
		return false;

	tm.stop();
	printf("\t%d + %d features.\t(%dms)\n", frameData1.getNbFeatures(), frameData2.getNbFeatures(), tm.duration());
	fflush(stdout);

	if (frameData1.getNbFeatures()==0)
		return false;
	if (frameData2.getNbFeatures()==0)
		return false;

	// ---------------------------------------------------------------------------
	// feature matching through kd-tree search
	// ---------------------------------------------------------------------------
	kdSearchFeatureMatches(
		frameData1,
		frameData2,
		indexMatches,
		matchesSource,
		matchesTarget,
		forLoopClosure);

	if (indexMatches.size()>3) {
		// ---------------------------------------------------------------------------
		//  find transformation with RANSAC iterations
		// ---------------------------------------------------------------------------
		validTransform = findTransformRANSAC(
				frameData1,
				frameData2,
				indexMatches,
				matchesSource,
				matchesTarget,
				resultingTransform,
				forLoopClosure);

		// ---------------------------------------------------------------------------
		//  refine transformation with ICP
		// ---------------------------------------------------------------------------
		if (validTransform && Config::_MatchingRunICP) {
			Eigen::Matrix4f icpTransform;
			// initial guess
			icpTransform = resultingTransform._matrix;
			// before ICP
			cout << resultingTransform._matrix << "\n";
			if (PointCloud::getTransformICP(frameData1, frameData2, icpTransform)) 	{
				// update the transformation
				resultingTransform._matrix= icpTransform;
			}
		}
	}

	return validTransform;
}

// -----------------------------------------------------------------------------------------------------
//  checkLoopClosure
// -----------------------------------------------------------------------------------------------------
bool checkLoopClosure(
		int frameID1,
		int frameID2,
		FrameData &frameData1,
		FrameData &frameData2,
		Transformation &resultingTransform)
{
	return computeTransformation(
			 frameID1,
			 frameID2,
			 frameData1,
			 frameData2,
			 resultingTransform,
			 true);
}

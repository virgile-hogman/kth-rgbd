#include <Eigen/Geometry>
#include "pcl/common/transformation_from_correspondences.h"

// Open CV
//#include "cxcore.h"
#include "cv.h"
#include "highgui.h"

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
#include "FeatureMatching.h"
#include "Timer.h"

#include <iostream>
#include <vector>

using namespace std;

/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 200
/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.49


#define MATCH_RELATIVE_DEPTH	0.1	// relative difference of depth for a valid match (higher -> more tolerant)

#define NB_RANSAC_ITERATIONS	20		// number of RANSAC iterations (loops)
#define MIN_NB_INLIERS_ABS		10		// minimum number of inliers
#define MIN_NB_INLIERS_REL		0.3		// minimum rate of inliers relatively to the initial matches

#define MAX_INLIER_DISTANCE		0.04	// error tolerance for inliers transformation (higher -> more tolerant)



// -----------------------------------------------------------------------------------------------------
//  computeInliersAndError
// -----------------------------------------------------------------------------------------------------
void computeInliersAndError(
		const Eigen::Matrix4f& transformation,
        const std::vector<Eigen::Vector3f>& matches_orig,
        const std::vector<Eigen::Vector3f>& matches_dest,
        std::vector<int>& inliers, //output var
        double& mean_error,
        double max_inlier_error_in_m)
{
    vector<pair<float,int> > dists;
    std::vector<int> inliers_temp;

    inliers.clear();
    mean_error = 0.0;
    
    for (unsigned int id = 0; id < matches_orig.size(); id++){ //compute new error and inliers

        // vectors with homogeneous coordinates
        Eigen::Vector4f orig(matches_orig[id][0], matches_orig[id][1], matches_orig[id][2], 1.0); 
        Eigen::Vector4f dest(matches_dest[id][0], matches_dest[id][1], matches_dest[id][2], 1.0);

        // project the point and compute the difference wrt the match
        Eigen::Vector4f vec = (transformation * orig) - dest;

        double error = vec.dot(vec);

        if (error > max_inlier_error_in_m) 
            continue; //ignore outliers

        error = sqrt(error);
        dists.push_back(pair<float,int>(error,id));
        inliers_temp.push_back(id); //include inlier

        mean_error += error;
    }

    if (inliers_temp.size()==0){
        mean_error = -1;
        inliers.clear();
    }
    else
    {
        mean_error /= inliers_temp.size();

        // sort inlier ascending according to their error
        sort(dists.begin(),dists.end());

        inliers.resize(inliers_temp.size());
        for (unsigned int i=0; i<inliers_temp.size(); i++){
            inliers[i] = dists[i].second;
        }
    }
}

// -----------------------------------------------------------------------------------------------------
//  matchFrames
// -----------------------------------------------------------------------------------------------------
bool matchFrames(
		int frameID1,
		int frameID2,
		FrameData &frameData1,
		FrameData &frameData2,
		bool methodRandom,
		Transformation &resultingTransfo)
{
    Timer tm;
	char buf[256];
	IplImage* imgStacked = NULL;
	CvFont font;
	double hScale=0.5;
	double vScale=0.5;
	int    lineWidth=1;
	
	bool validTransformation = false;
	
	int maxDeltaDepthArea=50;
	int sizeFeatureArea=-1;
	
	
	// define a font to write some text
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale,vScale, 0, lineWidth);
	
	resultingTransfo._matrix = Eigen::Matrix4f::Identity();
	resultingTransfo._error = 1.0;
	resultingTransfo._idOrig = frameID1;
	resultingTransfo._idDest = frameID2;
	
	// ---------------------------------------------------------------------------
	// feature detection
	// ---------------------------------------------------------------------------
    tm.start();
	printf("Frames %03d-%03d:\t Extracting SIFT features... ", frameID1, frameID2);
	fflush(stdout);
	
	// load data Frame1
	if (! frameData1.isLoaded(frameID1))
	{
		if (!frameData1.loadImage(frameID1))
			return false;
		if (!frameData1.loadDepthData())
			return false;
		
		frameData1.computeFeatures();
		frameData1.removeInvalidFeatures(sizeFeatureArea, maxDeltaDepthArea);
		frameData1.drawFeatures(font);
	}
	
	// load data Frame2
	if (! frameData2.isLoaded(frameID2))
	{
		if (!frameData2.loadImage(frameID2))
			return false;
		if (!frameData2.loadDepthData())
			return false;
		
		frameData2.computeFeatures();
		frameData2.removeInvalidFeatures(sizeFeatureArea, maxDeltaDepthArea);		
		frameData2.drawFeatures(font);		
	}

	// stack the 2 images
	imgStacked = stack_imgs( frameData1.getImage(), frameData2.getImage() );
				
    tm.stop();
	printf("\t%d + %d features.\t(%dms)\n", frameData1.getNbFeatures(), frameData2.getNbFeatures(), tm.duration());
	fflush(stdout);
	
	if (frameData1.getNbFeatures()==0)
		return false;
	if (frameData2.getNbFeatures()==0)
		return false;
	
	// ---------------------------------------------------------------------------
	// feature matching Kd-tree search
	// ---------------------------------------------------------------------------
	tm.start();
    fprintf( stderr, "Frames %03d-%03d:\t Searching for matches... ", frameID1, frameID2 );
	fflush(stdout);
	
	struct feature** neighbour_features = NULL;
	struct kd_node* kd_root;
	CvPoint pt1, pt2;
	double d0, d1;
	int k, i, nb_matches = 0, nb_valid_matches = 0;
	struct feature* feat;
	vector<int> index_matches;
	vector<Eigen::Vector3f>	vector_matches_orig;
	vector<Eigen::Vector3f>	vector_matches_dest;
	vector<int>	indexArea1, indexArea2, indexArea3;
    
	float constant = 0.001 / CameraDevice::_FocalLength;    // TODO - redefine this properly
	
	// try match the new features found in the 2d frame...
	kd_root = kdtree_build( frameData2.getFeatures(), frameData2.getNbFeatures() );
	for( i=0; i < frameData1.getNbFeatures(); i++ )
	{
		// ... looking in the 1st frame
		feat = frameData1.getFeatures() + i;
		// search for 2 nearest neighbours
		k = kdtree_bbf_knn( kd_root, feat, 2, &neighbour_features, KDTREE_BBF_MAX_NN_CHKS );
		if( k == 2 )
		{
			// the neighbours are ordered in increasing descriptor distance
			d0 = descr_dist_sq( feat, neighbour_features[0] );
			d1 = descr_dist_sq( feat, neighbour_features[1] );
			// check if the 2 are close enough
			if( d0 < d1 * NN_SQ_DIST_RATIO_THR )
			{
	            const struct feature* feat1 = frameData1.getFeature(i);
	            const struct feature* feat2 = neighbour_features[0];
	            
				// draw a line through the 2 points in the stacked image
				pt1 = cvPoint( cvRound( feat1->x ), cvRound( feat1->y ) );
				pt2 = cvPoint( cvRound( feat2->x ), cvRound( feat2->y ) );
				pt2.y += frameData1.getImage()->height;
				nb_matches++;
				
				// read depth info
				const TDepthPixel depth1 = frameData1.getFeatureDepth(feat1);
				const TDepthPixel depth2 = frameData2.getFeatureDepth(feat2);				
				
				// check if depth values are close enough (values in mm)
				if (depth1>0 && //depth1<2000 &&
					depth2>0 && //depth2<2000 &&
					abs(depth1-depth2)/float(depth1) < MATCH_RELATIVE_DEPTH)	// read: relative diff 
				{
					// draw a green line
					cvLine( imgStacked, pt1, pt2, CV_RGB(0,255,0), 1, 8, 0 );
					// this is a valid match
					nb_valid_matches++;
					// fwd link the previous features to the new features according to the match
					frameData1.setFeatureMatch(i, neighbour_features[0]);
					index_matches.push_back(i);
			
					// convert pixels to metric
					float z1 = (feat1->x - NBPIXELS_X_HALF) * depth1 * constant;
					float y1 = (NBPIXELS_Y_HALF - feat1->y) * depth1 * constant;
					float x1 = depth1 * 0.001 ; // given depth values are in mm
					
					float z2 = (feat2->x - NBPIXELS_X_HALF) * depth2 * constant;
					float y2 = (NBPIXELS_Y_HALF - feat2->y) * depth2 * constant;
					float x2 = depth2 * 0.001 ; // given depth values are in mm

					Eigen::Vector3f orig(x1,y1,z1);
					Eigen::Vector3f dest(x2,y2,z2);
					
					vector_matches_orig.push_back(orig);
					vector_matches_dest.push_back(dest);
					
					if (feat1->x < 210)
						indexArea1.push_back(nb_valid_matches-1);
					else if (feat1->x > 430)
						indexArea3.push_back(nb_valid_matches-1);
					else
						indexArea2.push_back(nb_valid_matches-1);
				}
				else
				{
					// ignore the pairs without depth any info, but show the remaining outliners
					if (depth1>0 || depth2>0)
					{
						char bufDiff[256];
						// draw a red line
						cvLine(imgStacked, pt1, pt2, CV_RGB(255,0,0), 1, 8, 0);
						//cvPutText(imgStacked, bufDiff, cvPoint((pt1.x+pt2.x)/2 -20,(pt1.y+pt2.y)/2), &font, cvScalar(255,255,0));
						sprintf(bufDiff,"_%u", depth1);
						cvPutText(imgStacked, bufDiff, cvPoint(pt1.x, pt1.y), &font, cvScalar(255,255,0));
						sprintf(bufDiff,"_%u", depth2);
						cvPutText(imgStacked, bufDiff, cvPoint(pt2.x, pt2.y), &font, cvScalar(255,255,0));
					}
				}
			}
		}
		free( neighbour_features );
	}
	
	// free memory
	kdtree_release( kd_root );
	
	// debug
	//const XnDepthPixel* p1 = g_arrayMD[frameID1-1].Data();
	//fprintf(stderr,"Center Depth Pixel = %u\n", p1[640*240 + 320]);
		
	// save stacked image
	sprintf(buf,"Matches:%d/%d (%d%%)", nb_valid_matches, nb_matches, nb_valid_matches*100/nb_matches);
	cvPutText(imgStacked, buf, cvPoint(5, 950), &font, cvScalar(255,255,0));
	sprintf(buf, "%s/sift_%d_%d.bmp", Config::_ResultDirectory.c_str(), frameID1, frameID2);
	cvSaveImage(buf, imgStacked);
	cvReleaseImage(&imgStacked);

	tm.stop();
    fprintf( stderr, "\tKeeping %d/%d matches.\t(%dms)\n", nb_valid_matches, nb_matches, tm.duration() );
	fflush(stdout);

		
	// ---------------------------------------------------------------------------
	//  find transformation through RANSAC iterations 
	// ---------------------------------------------------------------------------
	if (nb_valid_matches>3)
	{
		// find transform pairs
	    pcl::TransformationFromCorrespondences tfc;
        Eigen::Matrix4f best_transformation;
        std::vector<int> index_best_inliers;
        double best_error = 0.1;
        
        vector<int>	initialPairs;	// just to track the 3 first points
	    
		for (int iteration=0; iteration<NB_RANSAC_ITERATIONS ; iteration++)
		{
			//fprintf(stderr, "\nIteration %d ... \t", iteration+1);
			tfc.reset();
			if (methodRandom)
			{
				// pickup 3 points from matches
				for (int k = 0; k < 3; k++) {
					int id_match = rand() % nb_valid_matches;
					//int index1 = index_matches[id_match];
					tfc.add(vector_matches_orig[id_match], vector_matches_dest[id_match]);
				}
			}
			else
			{
				int id_match;
				initialPairs.clear();
				if (indexArea1.size()==0 || indexArea2.size()==0 || indexArea3.size()==0)
				{
					fprintf(stderr, "Data not dispatched. %d %d %d", indexArea1.size(), indexArea2.size(), indexArea3.size());
					break;
				}
				// select 1 random point from area1
				id_match = indexArea1[rand() % indexArea1.size()];
				tfc.add(vector_matches_orig[id_match], vector_matches_dest[id_match]);
				initialPairs.push_back(index_matches[id_match]);
				// select 1 random point from area2
				id_match = indexArea2[rand() % indexArea2.size()];
				tfc.add(vector_matches_orig[id_match], vector_matches_dest[id_match]);
				initialPairs.push_back(index_matches[id_match]);
				// select 1 random point from area3
				id_match = indexArea3[rand() % indexArea3.size()];
				tfc.add(vector_matches_orig[id_match], vector_matches_dest[id_match]);
				initialPairs.push_back(index_matches[id_match]);
			}
			
	        // compute transformation from matches
	        Eigen::Matrix4f transformation = tfc.getTransformation().matrix();
	        
	        // compute error and keep only inliers
	        std::vector<int> index_inliers;
	        double mean_error;
	        double max_inlier_distance_in_m = MAX_INLIER_DISTANCE;
	        
	        computeInliersAndError(transformation,
	        		vector_matches_orig,
	        		vector_matches_dest,
	        		index_inliers,
	        		mean_error,
	        		max_inlier_distance_in_m * max_inlier_distance_in_m);
	        
	        //fprintf(stderr, "Found %d inliers\tMean error:%f", index_inliers.size(), mean_error);
	        
	        if (mean_error<0 || mean_error >= max_inlier_distance_in_m)
	        	continue;	// skip these 3 points and go for a new iteration
	        	
	        if (index_inliers.size()<MIN_NB_INLIERS_ABS || index_inliers.size()<nb_valid_matches*MIN_NB_INLIERS_REL)
	        	continue;	// not enough inliers found
	        
			if (mean_error < best_error)
	        {
		        //fprintf(stderr, "\t => Best candidate transformation! ", index_inliers.size(), mean_error);
	        	best_transformation = transformation;
	        	best_error = mean_error;
	        	index_best_inliers = index_inliers;
	        }
	        
			// ----------------------------------------------------
			// recompute a new transformation with the inliers
			// ----------------------------------------------------
			//fprintf(stderr, "\nRecomputing transfo... \t");
			tfc.reset();
			//for (int k = 0; k < 3; k++) {
			//    int id_inlier = rand() % index_inliers.size();
			//for (int id_inlier = 0; id_inlier < 3; id_inlier++) {	// the 3 best	        
			for (int id_inlier = 0; id_inlier < index_inliers.size(); id_inlier++) {
				int id_match  = index_inliers[id_inlier];
				tfc.add(vector_matches_orig[id_match], vector_matches_dest[id_match]);
			}
			// compute transformation from inliers
			transformation = tfc.getTransformation().matrix();
			
			computeInliersAndError(transformation,
					vector_matches_orig,
					vector_matches_dest,
					index_inliers,
					mean_error,
					max_inlier_distance_in_m * max_inlier_distance_in_m);
			
			if (mean_error<0 || mean_error >= max_inlier_distance_in_m)
				continue;	// skip these 3 points and go for a new iteration
				
			if (index_inliers.size()<MIN_NB_INLIERS_ABS || index_inliers.size()<nb_valid_matches*MIN_NB_INLIERS_REL)
				continue;	// not enough inliers found
			
			//fprintf(stderr, "Found %d inliers\tMean error:%f", index_inliers.size(), mean_error);
			
			if (mean_error < best_error)
			{
				//fprintf(stderr, "\t => Best transformation! ", index_inliers.size(), mean_error);
				best_transformation = transformation;
				best_error = mean_error;
				index_best_inliers = index_inliers;
			}
		}
		fprintf(stderr, "\n");
		
		
		if (index_best_inliers.size()>0)
		{
			// SUCCESS - TRANSFORMATION IS DEFINED
			
			// print the transformation matrix
			std::cerr << "Best Transformation --->\t" << index_best_inliers.size() << " inliers and mean error="<< best_error << std::endl;
			std:cerr << best_transformation << std::endl;
			fflush(stderr);

			resultingTransfo._matrix = best_transformation;
			resultingTransfo._error = best_error;
			validTransformation = true;
			
			// draw inliers
			IplImage* stacked_inliers = NULL;
			
			// stack the 2 images
			stacked_inliers = stack_imgs( frameData1.getImage(), frameData2.getImage() );
						
			// draw red lines for outliers
			// all the initial matches are drawn here - the inliers will be overwritten with green
			for (int i=0; i<index_matches.size(); i++)
			{
				int id_match = index_matches[i];
	            const struct feature* feat1 = frameData1.getFeature(id_match);
	            const struct feature* feat2 = frameData1.getFeatureMatch(id_match);
				
				// draw a line through the 2 points in the stacked image
				pt1 = cvPoint( cvRound( feat1->x ), cvRound( feat1->y ) );
				pt2 = cvPoint( cvRound( feat2->x ), cvRound( feat2->y ) );
				pt2.y += frameData1.getImage()->height;
				// draw a green line
				cvLine( stacked_inliers, pt1, pt2, CV_RGB(255,0,0), 1, 8, 0 );
			}
			// draw green lines for the best inliers
			for (int i=0; i<index_best_inliers.size(); i++)
			{
				int id_inlier = index_best_inliers[i];
				int id_match = index_matches[id_inlier];
	            const struct feature* feat1 = frameData1.getFeature(id_match);
	            const struct feature* feat2 = frameData1.getFeatureMatch(id_match);
				
				// draw a line through the 2 points in the stacked image
				pt1 = cvPoint( cvRound( feat1->x ), cvRound( feat1->y ) );
				pt2 = cvPoint( cvRound( feat2->x ), cvRound( feat2->y ) );
				pt2.y += frameData1.getImage()->height;
				// draw a green line
				cvLine( stacked_inliers, pt1, pt2, CV_RGB(0,255,0), 1, 8, 0 );
			}
			fprintf(stderr, "\n"); fflush(stderr);
			
			// draw lines for the 3 initial points
			for (int i=0; i<initialPairs.size(); i++)
			{
				int id_match = initialPairs[i];
	            const struct feature* feat1 = frameData1.getFeature(id_match);
	            const struct feature* feat2 = frameData1.getFeatureMatch(id_match);
				
				// draw a line through the 2 points in the stacked image
				pt1 = cvPoint( cvRound( feat1->x ), cvRound( feat1->y ) );
				pt2 = cvPoint( cvRound( feat2->x ), cvRound( feat2->y ) );
				pt2.y += frameData1.getImage()->height;
				// draw a green line
				cvLine( stacked_inliers, pt1, pt2, CV_RGB(0,0,140), 2, 8, 0 );
			}
			
			// save stacked image
			sprintf(buf,"inliers:%d/%d (%d%%)", index_best_inliers.size(), nb_valid_matches, index_best_inliers.size()*100/nb_valid_matches);
			cvPutText(stacked_inliers, buf, cvPoint(5, 950), &font, cvScalar(255,255,0));
			sprintf(buf, "%s/sift_%d_%d_inliers.bmp", Config::_ResultDirectory.c_str(), frameID1, frameID2);
			cvSaveImage(buf, stacked_inliers);
			cvReleaseImage(&stacked_inliers);
		}
	}
	
	return validTransformation;
}


///////////////////
// SIFT EXTRACTION
///////////////////

#include <Eigen/Geometry>
#include "pcl/common/transformation_from_correspondences.h"
/*
*/
/*#include "opencv/cv.h"
#include "opencv/highgui.h"*/
// PCL includes
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/common/transform.h"
#include "pcl/filters/voxel_grid.h"

// Open CV
//#include "cxcore.h"
#include "cv.h"
#include "highgui.h"
#include <boost/filesystem.hpp>

extern "C" {
#include "sift.h"
#include "imgfeatures.h"
#include "kdtree.h"
#include "utils.h"	// stack_imgs
#include "xform.h"
}
//#include "extract.h"

#include "CommonTypes.h"
#include "FrameData.h"
#include "Timer.h"
#include "Graph.h"

// Open NI
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>

// standard
#include <stdio.h>
#include <iostream>

#include <vector>
#include <list>

using namespace xn;
using namespace std;


#define CHECK_RC(rc, what)                                            \
    if (rc != XN_STATUS_OK)                                            \
{                                                                \
    printf("%s failed: %s\n", what, xnGetStatusString(rc));        \
    return rc;                                                    \
}

#define OPENNI_CONFIG_XML_PATH "SamplesConfig.xml"

/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 200
/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.49


#define MATCH_RELATIVE_DEPTH	0.05	// relative difference of depth for a valid match (higher -> more tolerant)

#define NB_RANSAC_ITERATIONS	20		// number of RANSAC iterations (loops)
#define MIN_NB_INLIERS_ABS		10		// minimum number of inliers
#define MIN_NB_INLIERS_REL		0.6		// minimum rate of inliers relatively to the initial matches

#define MAX_INLIER_DISTANCE		0.04	// error tolerance for inliers transformation (higher -> more tolerant)


#define MAX_DEPTH_HISTOGRAM 10000		// previously used for histogram only

const double rgb_focal_length_VGA = 525;

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
Graph g_graph;

DepthGenerator g_depth;
ImageGenerator g_image;
DepthMetaData g_depthMD;
ImageMetaData g_imageMD;
Context g_context;
XnFPSData xnFPS;
XnUInt64 no_sample_value, shadow_value;

// working buffers reused for each frame (just to avoid reallocate the arrays each time) 
IplImage* g_imgRGB = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
IplImage* g_imgDepth = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);

std::string g_dataDirectory = "data_in";
std::string g_genDirectory = "data_gen";
std::string g_resultDirectory = "data_out";

// PCL
pcl::PointCloud<pcl::PointXYZRGB> g_cloudPointSave;

// -----------------------------------------------------------------------------------------------------
//  common data types
// -----------------------------------------------------------------------------------------------------
float bad_point = std::numeric_limits<float>::quiet_NaN ();


// -----------------------------------------------------------------------------------------------------
//  connectKinect
// -----------------------------------------------------------------------------------------------------
XnStatus connectKinect()
{
    //Connect to kinect
    printf("Connecting to Kinect... ");
    fflush(stdout);
    XnStatus nRetVal = XN_STATUS_OK;
    EnumerationErrors errors;
    nRetVal = g_context.InitFromXmlFile(OPENNI_CONFIG_XML_PATH, &errors);
    if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
    {
        XnChar strError[1024];
        errors.ToString(strError, 1024);
        printf("%s\n", strError);
        return (nRetVal);
    }
    else if (nRetVal != XN_STATUS_OK)
    {
        printf("Open failed: %s\n", xnGetStatusString(nRetVal));
        return (nRetVal);
    }
    printf("OK\n");   
    
    nRetVal = g_context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_depth);
    CHECK_RC(nRetVal, "Find depth generator");

    nRetVal = g_context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_image);
    CHECK_RC(nRetVal, "Find image generator");

    nRetVal = xnFPSInit(&xnFPS, 180);
    CHECK_RC(nRetVal, "FPS Init");

    g_context.SetGlobalMirror(true); //mirror image 

    g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);
    if (g_depth.GetIntProperty ("ShadowValue", shadow_value) != XN_STATUS_OK)
      printf ("[OpenNIDriver] Could not read shadow value!");

    if (g_depth.GetIntProperty ("NoSampleValue", no_sample_value) != XN_STATUS_OK)
      printf ("[OpenNIDriver] Could not read no sample value!");

    return nRetVal;
}


// -----------------------------------------------------------------------------------------------------
//  saveRGBImage
// -----------------------------------------------------------------------------------------------------
void saveRGBImage(const XnRGB24Pixel* pImageMap, IplImage* tmp_img = 0, bool doSave = false){

    // Convert to IplImage 24 bit, 3 channels
    for(unsigned int i = 0; i < g_imageMD.XRes()*g_imageMD.YRes();i++)
    {
        tmp_img->imageData[3*i+0]=pImageMap[i].nBlue;
        tmp_img->imageData[3*i+1]=pImageMap[i].nGreen;
        tmp_img->imageData[3*i+2]=pImageMap[i].nRed;
    }
    if (doSave){
        char buf[256];
        sprintf(buf, "%s/frame_%d_rgb.bmp", g_dataDirectory.c_str(), g_depthMD.FrameID());
        cvSaveImage(buf, tmp_img);
    }
}

// -----------------------------------------------------------------------------------------------------
//  saveHistogramImage
// -----------------------------------------------------------------------------------------------------
int saveHistogramImage(
		const XnRGB24Pixel* pImageMap,
		const XnDepthPixel* pDepthMap,
		IplImage* pImgDepth)
{
	static float depthHistogram[MAX_DEPTH_HISTOGRAM];
	
	// Calculate the accumulative histogram (the yellow display...)
	const XnDepthPixel* pDepth = g_depthMD.Data();    
	xnOSMemSet(depthHistogram, 0, MAX_DEPTH_HISTOGRAM*sizeof(float));
	unsigned int nNumberOfPoints = 0;
	// count depth values
	for (XnUInt y = 0; y < g_depthMD.YRes(); ++y)
	{
		for (XnUInt x = 0; x < g_depthMD.XRes(); ++x, ++pDepth)
		{
			if (*pDepth != 0)
			{
				depthHistogram[*pDepth]++;
				nNumberOfPoints++;
			}
		}
	}
	// cumulative sum
	for (int nIndex=1; nIndex<MAX_DEPTH_HISTOGRAM; nIndex++)
	{
		depthHistogram[nIndex] += depthHistogram[nIndex-1];
	}
	// rescale to 0..256
	if (nNumberOfPoints)
	{
		for (int nIndex=1; nIndex<MAX_DEPTH_HISTOGRAM; nIndex++)
		{
			depthHistogram[nIndex] = (unsigned int)(256 * (1.0f - (depthHistogram[nIndex] / nNumberOfPoints)));
		}
	}
	// generate histogram depth image
	int i = 0;
	pDepth = g_depthMD.Data();
	for (XnUInt y = 0; y < g_depthMD.YRes(); ++y)
	{
		for (XnUInt x = 0; x < g_depthMD.XRes(); ++x, ++pDepth, ++i)
		{
			unsigned char nHistValue = 0;
			
			if (*pDepth != 0)
				nHistValue = depthHistogram[*pDepth];
			
			// yellow pixels
			pImgDepth->imageData[3*i+0] = 0;			//Blue
			pImgDepth->imageData[3*i+1] = nHistValue;	//Green
			pImgDepth->imageData[3*i+2] = nHistValue;	//Red
		}
	}
	
	char bufFilename[256];
	sprintf(bufFilename,"%s/frame_%d_histo.bmp", g_dataDirectory.c_str(), g_depthMD.FrameID());
	cvSaveImage(bufFilename, pImgDepth);
}

// -----------------------------------------------------------------------------------------------------
//  saveDepthImage
// -----------------------------------------------------------------------------------------------------
int saveDepthImage(
		const XnRGB24Pixel* pImageMap,
		const XnDepthPixel* pDepthMap,
		IplImage* pImgDepth,
		bool savePointCloud)
{
	//printf("Nb Channels: %d depth:%d/%d\n", pImgDepth->nChannels, pImgDepth->depth, IPL_DEPTH_16U);
	//fflush(stdout);
	
	// Save only the Z value per pixel as an image for quick visualization of depth
	for(int i = 0; i < g_depthMD.XRes()*g_depthMD.YRes();i++)
	{
		// depth pixels on 16 bits
		//short depthValue = pDepthMap[i]/16;	// for quick look only
		pImgDepth->imageData[3*i+0]=(unsigned char)(pDepthMap[i]>>8);
		pImgDepth->imageData[3*i+1]=(unsigned char)(pDepthMap[i] & 0xFF);
		pImgDepth->imageData[3*i+2]=0;
		//pImgDepth->imageData[i] = pDepthMap[i];
	}
	//printf("Depth value saved at (320,240):%x \t%x\n", pDepthMap[MIDDLE_POINT], (unsigned short)pImgDepth->imageData[MIDDLE_POINT]);	
	
	// save the depth image
	char bufFilename[256];
	sprintf(bufFilename, "%s/frame_%d_depth.bmp", g_dataDirectory.c_str(), g_depthMD.FrameID());
	cvSaveImage(bufFilename, pImgDepth);
    
	// point cloud
    if (savePointCloud)
    {
		float constant = 0.001 / rgb_focal_length_VGA;    
		unsigned int rgb; 
		int depth_index = 0;
		int ImageCenterX = g_depthMD.XRes() >> 1;
		int ImageCenterY = g_depthMD.YRes() >> 1;
		for (int ind_y =0; ind_y < g_depthMD.YRes(); ind_y++)
		{
			for (int ind_x=0; ind_x < g_depthMD.XRes(); ind_x++, depth_index++)
			{
				pcl::PointXYZRGB& pt = g_cloudPointSave(ind_x,ind_y);
		
				if (pDepthMap[depth_index] == no_sample_value ||
					pDepthMap[depth_index] == shadow_value ||
					pDepthMap[depth_index] == 0 ){
		
					pt.x = bad_point;
					pt.y = bad_point;
					pt.z = bad_point;
				}
				else 
				{
					// locate point in meters
					pt.x = (ind_x - ImageCenterX) * pDepthMap[depth_index] * constant;
					pt.y = (ImageCenterY - ind_y) * pDepthMap[depth_index] * constant;
					pt.z = pDepthMap[depth_index] * 0.001 ; // given depth values are in mm
					rgb = (((unsigned int)pImageMap[depth_index].nRed) << 16) |
						  (((unsigned int)pImageMap[depth_index].nGreen) << 8) |
						  ((unsigned int)pImageMap[depth_index].nBlue);
					pt.rgb = *reinterpret_cast<float*>(&rgb);
				}
			}
		}
		
		char buf[256];
		sprintf(buf, "%s/cloud%d.pcd", g_resultDirectory.c_str(), g_depthMD.FrameID());
		pcl::io::savePCDFile(buf, g_cloudPointSave, true);
		// bug in PCL - the binary file is not created with the good rights!
		char bufsys[256];
		sprintf(bufsys, "chmod a+rw %s", buf);
		system(bufsys);
    }
	
	return g_depthMD.FrameID();
}

// -----------------------------------------------------------------------------------------------------
//  generateFrames
// -----------------------------------------------------------------------------------------------------
void generateFrames(int nbRemainingFrames, vector<int> &framesID)
{
    XnStatus nRetVal = XN_STATUS_OK;
	
	framesID.clear();
	
	while (nbRemainingFrames > 0)
	{
		const XnDepthPixel* pDepthMap = NULL;
		const XnRGB24Pixel* pImageMap = NULL;
		
		xnFPSMarkFrame(&xnFPS);
		nRetVal = g_context.WaitAndUpdateAll();
		if (nRetVal == XN_STATUS_OK)
		{
			g_depth.GetMetaData(g_depthMD);
			g_image.GetMetaData(g_imageMD);
	
			pDepthMap = g_depthMD.Data();
			pImageMap = g_image.GetRGB24ImageMap();
			
			printf("Frame %02d (%dx%d) Depth at middle point: %u. FPS: %f\r",
					g_depthMD.FrameID(),
					g_depthMD.XRes(),
					g_depthMD.YRes(),
					g_depthMD(g_depthMD.XRes()/2, g_depthMD.YRes()/2),
					xnFPSCalc(&xnFPS));
		}
		if (xnOSWasKeyboardHit())
		{
			char c = xnOSReadCharFromInput();	// to reset the keyboard hit
			nbRemainingFrames--;

			saveRGBImage(pImageMap, g_imgRGB, true);
			int frameID = saveDepthImage(pImageMap, pDepthMap, g_imgDepth, false);
			//usleep(100000);
			
			printf("\nAdding frame %d in sequence, remaining: %d\n", frameID, nbRemainingFrames);
			framesID.push_back(frameID);
		}
	}
}

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
	
	int maxDeltaDepthArea=50;
	int sizeFeatureArea=-1;
	
	// define a font to write some text
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale,vScale, 0, lineWidth);
	
	resultingTransfo._isValid = false;
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
    
	float constant = 0.001 / rgb_focal_length_VGA;    // TODO - redefine this properly
	
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
					float x1 = (feat1->x - NBPIXELS_X_HALF) * depth1 * constant;
					float y1 = (NBPIXELS_Y_HALF - feat1->y) * depth1 * constant;
					float z1 = depth1 * 0.001 ; // given depth values are in mm
					
					float x2 = (feat2->x - NBPIXELS_X_HALF) * depth2 * constant;
					float y2 = (NBPIXELS_Y_HALF - feat2->y) * depth2 * constant;
					float z2 = depth2 * 0.001 ; // given depth values are in mm

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
	sprintf(buf, "%s/sift_%d_%d.bmp", g_resultDirectory.c_str(), frameID1, frameID2);
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

			resultingTransfo._isValid = true;
			resultingTransfo._matrix = best_transformation;
			resultingTransfo._error = best_error;
			
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
			sprintf(buf, "%s/sift_%d_%d_inliers.bmp", g_resultDirectory.c_str(), frameID1, frameID2);
			cvSaveImage(buf, stacked_inliers);
			cvReleaseImage(&stacked_inliers);
		}
	}
	
	return resultingTransfo._isValid;
}

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
	pointCloud.points.reserve(NBPIXELS_WIDTH*NBPIXELS_HEIGHT);	// memory allocation
	//pointCloud.points.resize(NBPIXELS_WIDTH*NBPIXELS_HEIGHT);
	
	//printf("Generating point cloud frame %d\n", frameID);
		
	float constant = 0.001 / rgb_focal_length_VGA;    
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
				pt.x = (ind_x - NBPIXELS_X_HALF) * depth * constant;
				pt.y = (NBPIXELS_Y_HALF - ind_y) * depth * constant;
				pt.z = depth * 0.001 ; // given depth values are in mm
				
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
//  buildMap
// -----------------------------------------------------------------------------------------------------
void buildMap(vector<int> &framesID, TransformationVector &poseTransformations, bool cumulateTransfo, bool savePointCloud, const char *filenamePCD)
{
	char buf_full[256];
	sprintf(buf_full, "%s/cloud_full.pcd", g_resultDirectory.c_str());
	pcl::PointCloud<pcl::PointXYZRGB> cloudFull;
	pcl::PointCloud<pcl::PointXYZRGB> cloudFrame;
	pcl::PointCloud<pcl::PointXYZRGB> cloudFrameTransformed;
	
	//Eigen::Vector4f cameraPose(0, 0, 0, 1.0); 
	Eigen::Matrix4f cumulatedTransformation = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f inverseTfo;
	bool valid_sequence = true;
	
	if (savePointCloud && poseTransformations.size()>0)
	{
		cout << "Initialize point cloud frame #" << framesID[0] << " (1/" << framesID.size() << ")..." << std::endl;
		//char buf[256];
		//sprintf(buf, "%s/cloud%d.pcd", g_dataDirectory.c_str(), framesID[0]);
		//if (pcl::io::loadPCDFile(buf, cloudFull) != 0)
		generatePointCloud(framesID[0], cloudFull);
	}
	
	for (int iPose=0; iPose<poseTransformations.size(); iPose++)
	{
		cout << "----------------------------------------------------------------------------" << std::endl;
		pcl::PointXYZRGB ptCameraPose;
		
		//cameraPose = tfoGraph._matrix.inverse() * Eigen::Vector4f(0,0,0,1);
		//cout << "camera pose graph\n" << cameraPose << std::endl;
		//cameraPose = poseTransformations[iPose]._matrix.inverse() * cameraPose;
		//cout << "camera pose std\n" << cameraPose << std::endl;
		
		// compute global transformation from the start
		if (cumulateTransfo)
			cumulatedTransformation = poseTransformations[iPose]._matrix * cumulatedTransformation;
		else
			cumulatedTransformation = poseTransformations[iPose]._matrix;
			
		cout << "camera pose std\n" << cumulatedTransformation.inverse() << std::endl;

		cout << "Mean error:" << poseTransformations[iPose]._error << std::endl;
		if (valid_sequence)
		{
			if (! poseTransformations[iPose]._isValid)
			{
				valid_sequence = false;
				cout << "--- Invalid sequence - aborting point cloud accumulation --- \n" << std::endl;
				// but the pose loop continues just to display the next transformations
			}
			
			// update global point cloud
			if (savePointCloud)
			{
				cout << "Generating point cloud frame #" << framesID[iPose+1] << " (" << iPose+2 << "/" << framesID.size() << ")...";
				//char buf[256];
				//sprintf(buf, "%s/cloud%d.pcd", g_dataDirectory.c_str(), framesID[iPose+1]);
				//if (pcl::io::loadPCDFile(buf, cloudFrame) != 0)
				generatePointCloud(framesID[iPose+1], cloudFrame);
	
				// subsample frame keeping 50%
				subsamplePointCloud(cloudFrame, 50);
				
				// inverse transform
				inverseTfo = cumulatedTransformation.inverse();
				
				// apply transformation to the point cloud
				pcl::getTransformedPointCloud(
						cloudFrame,
						Eigen::Affine3f(inverseTfo),
						cloudFrameTransformed); 
				
				// apend transformed point cloud
				cloudFull += cloudFrameTransformed;
				cout << " Total Size: " << cloudFull.size() << " points." << std::endl;
				
/*				if (iPose % 10 == 0 && iPose<poseTransformations.size()-10)
					subsamplePointCloud(cloudFull);*/
			}
		}
	}
	if (savePointCloud && cloudFull.size()>0)
	{
		// subsample final point cloud keeping 70%
		subsamplePointCloud(cloudFull, 70);

		// correct x-axis mirroring
		for (int i=0; i<cloudFull.size(); i++)
			cloudFull.points[i].x = -cloudFull.points[i].x;
		
		cout << "Saving global point cloud binary..." << std::endl;    			
		sprintf(buf_full, "%s/%s", g_resultDirectory.c_str(), filenamePCD);
		pcl::io::savePCDFile(buf_full, cloudFull, true);
		// bug in PCL - the binary file is not created with the good rights!
		char bufsys[256];
		sprintf(bufsys, "chmod a+rw %s", buf_full);
		system(bufsys);
	}
}

void loadSequence(const char *dataDirectory, int skipCount, int min, int max, vector<int> &sequenceFramesID)
{
	int frameID;
	list<int> listFramesID;
	
	sequenceFramesID.clear();
	
	boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
	for ( boost::filesystem::directory_iterator itr( dataDirectory );
		itr != end_itr;
		++itr )
	{
		//	printf("%s\t%s\n", itr->leaf().c_str(), itr->path().string().c_str());
		if (boost::filesystem::extension(*itr)==".bmp" &&
			sscanf(itr->leaf().c_str(), "frame_%d", &frameID)==1 && frameID>=min && (frameID<=max || max<0))
		{
			// add frame
			printf("Add frame file #%i:\t%s\n", frameID, itr->path().string().c_str());
			listFramesID.push_back(frameID);
		}
	}
	// sort and keep only 1 element 
	listFramesID.sort();
	listFramesID.unique();
	
	// build the sequence
	int counter = 0;
	cout << "Sequence of frames: (";
	while (!listFramesID.empty())
	{
		if (skipCount<=0 || counter==0 || counter>=skipCount)
		{
			cout << " " << listFramesID.front();
			sequenceFramesID.push_back(listFramesID.front());
			if (counter>0)
				counter = 0;
		}
		listFramesID.pop_front();
		counter++;
	}
	cout << ") for " << sequenceFramesID.size() << " frames." << std::endl;
}

void buildMapSequence(vector<int> &sequenceFramesID, bool savePointCloud)
{
	if (sequenceFramesID.size()>=2)
	{
		FrameData frameData1, frameData2;
		FrameData frameDataLoopClosure;
		TransformationVector resultingTransformations;
		Transformation transfo;
		bool retCode;
		Eigen::Matrix4f cameraPoseMat(Eigen::Matrix4f::Identity());

		// initialize the graph
	    g_graph.initialize();

	    // add vertex for the initial pose
		g_graph.addVertex(0, cameraPoseMat);
	    
		for (int iFrame=1; iFrame<sequenceFramesID.size(); iFrame++)
		{
			// match frame to frame (current with previous)
			retCode = matchFrames(
					sequenceFramesID[iFrame-1],
					sequenceFramesID[iFrame],
					frameData1,
					frameData2,
					true,
					transfo);
			
			if (!retCode)
				break;	// no valid transform
			
			resultingTransformations.push_back(transfo);
			
			cameraPoseMat = transfo._matrix * cameraPoseMat;
			std::cerr << "Camera mat\n" << cameraPoseMat << std::endl;

		    // add a new vertex
		    g_graph.addVertex(iFrame, cameraPoseMat);
	    
		    // add the edge = constraints
		    g_graph.addEdge(iFrame-1, iFrame, transfo);
		    
			/*if (iFrame>1 && iFrame%6 == 0)
			{
				// loop closure with frame #1
				retCode = matchFrames(
						sequenceFramesID[1],
						sequenceFramesID[iFrame],
						frameDataLoopClosure,
						frameData2,
						true,
						transfo);
				
				if (retCode)
				{
				    // add the edge = constraints
					std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
					std::cerr << " LOOP CLOSURE DETECTED " << std::endl;
					std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
				    g_graph.addEdge(1, iFrame, transfo);
				}
			}			*/
			
			// free data
			frameData1.releaseData();
			// reassign the last frame to avoid reloading all the data twice
			frameData1.assignData(frameData2);
		}
	    
		g_graph.setSaveDirectory(g_resultDirectory.c_str());
		g_graph.save("graph.g2o");
		
		// build map, generates point cloud
		buildMap(sequenceFramesID, resultingTransformations, true, savePointCloud, "cloud_full.pcd");
		
		g_graph.optimize();
		g_graph.save("graph_opt.g2o");
		
		// extract the transformations from the optimized graph
		resultingTransformations.clear();
		for (int iFrame=1; iFrame<sequenceFramesID.size(); iFrame++)
		{		
			Transformation tfoGraph;
			g_graph.getTransfo(iFrame, tfoGraph);
			
			if (! tfoGraph._isValid)
				break;
					
			resultingTransformations.push_back(tfoGraph);
			//cout << "camera pose graph\n" << cumulatedTransformationGraph.inverse() << std::endl;
		}
		buildMap(sequenceFramesID, resultingTransformations, false, savePointCloud, "cloud_full_opt.pcd");
		
		frameData1.releaseData();
		frameData2.releaseData();
	}
	
}

// -----------------------------------------------------------------------------------------------------
//  Main program
// -----------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
	bool savePointCloud = true;
	vector<int> sequenceFramesID;
	int nbFrames=2;	// by default
    
	if (argc<1)
	{
		printf("Usage: %s --load | --save <nbFrames>", argv[0]);
		return -1;
	}
	if (argc>2)
		nbFrames = atoi(argv[2]);
	
	if (argc>3 && atoi(argv[3])<=0)
		savePointCloud = false;
	
    printf("Start\n");
    
    FrameData::_DataPath = g_dataDirectory;
    
    if (strcmp(argv[1], "--load") == 0)
    {
    	// load sequence from directory
    	if ( ! boost::filesystem::exists( g_dataDirectory ) )
    		return -1;

    	int skip=0, min=-1, max=-1;
    	if (argc>4)
    		skip = atoi(argv[4]);
    	if (argc>3)
    		max = atoi(argv[3]);
    	if (argc>2)
    		min = atoi(argv[2]);
    	
    	loadSequence(g_dataDirectory.c_str(), skip, min, max, sequenceFramesID);
    	
        boost::filesystem::create_directories(g_resultDirectory);       
		
    	// build map 
    	buildMapSequence(sequenceFramesID, savePointCloud);
	}
    
    if (strcmp(argv[1], "--save") == 0)
    {
        XnStatus nRetVal = XN_STATUS_OK;
        
        // use the same directory to generate and reload the data
        g_dataDirectory = g_genDirectory;
        FrameData::_DataPath = g_genDirectory;
    		
    	if (nbFrames<2)
    	{
    		printf("At least 2 frames are required!\n");    		
    		return -1;
    	}
        boost::filesystem::create_directories(g_dataDirectory);       

        nRetVal = connectKinect();
        if (nRetVal == XN_STATUS_OK)
        {
        	// allocate the point cloud buffer
        	g_cloudPointSave.width = 640;
            g_cloudPointSave.height = 480;
            g_cloudPointSave.points.resize(640*480);
        	
            boost::filesystem::create_directories(g_resultDirectory);
            
        	// generate n frames and get its sequence
        	generateFrames(nbFrames, sequenceFramesID);
        
        	// build map 
        	buildMapSequence(sequenceFramesID, savePointCloud);
        	
            g_context.Shutdown();
        }
    }
    else {
    	printf("Select a valid option");
    	return -1;
    }
    
    return 0;
}

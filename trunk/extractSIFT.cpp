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

// Open NI
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>

// standard
#include <stdio.h>
#include <iostream>

#include <vector>
#include <Eigen/StdVector>

using namespace xn;
using namespace std;


#define CHECK_RC(rc, what)                                            \
    if (rc != XN_STATUS_OK)                                            \
{                                                                \
    printf("%s failed: %s\n", what, xnGetStatusString(rc));        \
    return rc;                                                    \
}

#define SAMPLE_XML_PATH "SamplesConfig.xml"

#define NBPIXELS_X_HALF 320
#define NBPIXELS_Y_HALF 240

/* the maximum number of keypoint NN candidates to check during BBF search */
#define KDTREE_BBF_MAX_NN_CHKS 200
/* threshold on squared ratio of distances between NN and 2nd NN */
#define NN_SQ_DIST_RATIO_THR 0.49

const double rgb_focal_length_VGA = 525;

#define NB_RANSAC_ITERATIONS 20
#define MIN_NB_INLIERS		10

//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
DepthGenerator g_depth;
ImageGenerator g_image;
DepthMetaData g_depthMD;
ImageMetaData g_imageMD;
Context g_context;
XnFPSData xnFPS;
XnUInt64 no_sample_value, shadow_value;

vector<DepthMetaData*> g_vectDepthMD;
//DepthMetaData g_arrayMD[50];

#define MAX_DEPTH 10000
float g_pDepthHist[MAX_DEPTH];
XnRGB24Pixel* g_pTexMap = NULL;
unsigned int g_nTexMapX = 0;
unsigned int g_nTexMapY = 0;

IplImage* rgb_data=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
IplImage* depth_data=cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);

IplImage* save_img_rgb = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
IplImage* save_img_depth = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);

std::string saveFolderName = "data";

// PCL
float bad_point = std::numeric_limits<float>::quiet_NaN ();
pcl::PointCloud<pcl::PointXYZRGB> cloud_save;


typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue; // Blue channel
    unsigned char Green; // Green channel
    unsigned char Red; // Red channel
    unsigned char Alpha; // Alpha channel
  };
  float float_value;
  long long_value;
} RGBValue;


// -----------------------------------------------------------------------------------------------------
//  Transformations
// -----------------------------------------------------------------------------------------------------
class PoseTransformation
{
public:
	bool			_is_valid;
	Eigen::Matrix4f	_matrix;
	double			_error;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// for alignment read http://eigen.tuxfamily.org/dox/UnalignedArrayAssert.html
};

vector<PoseTransformation, Eigen::aligned_allocator<Eigen::Vector4f> > g_transformations;

//vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Vector4f> > g_transformations;


// -----------------------------------------------------------------------------------------------------
//  Timer for performance tracker
// -----------------------------------------------------------------------------------------------------
#include <cstdlib>
#include <sys/time.h>

class Timer
{
    timeval timer[2];

  public:

    timeval start()
    {
        gettimeofday(&this->timer[0], NULL);
        return this->timer[0];
    }

    timeval stop()
    {
        gettimeofday(&this->timer[1], NULL);
        return this->timer[1];
    }

    int duration() const
    {
        int secs(this->timer[1].tv_sec - this->timer[0].tv_sec);
        int usecs(this->timer[1].tv_usec - this->timer[0].tv_usec);

        if(usecs < 0)
        {
            --secs;
            usecs += 1000000;
        }

        return static_cast<int>(secs * 1000 + usecs / 1000.0 + 0.5);
    }
};


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
    nRetVal = g_context.InitFromXmlFile(SAMPLE_XML_PATH, &errors);
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

    g_context.SetGlobalMirror(false); //mirror image 

    g_depth.GetAlternativeViewPointCap().SetViewPoint(g_image);
    if (g_depth.GetIntProperty ("ShadowValue", shadow_value) != XN_STATUS_OK)
      printf ("[OpenNIDriver] Could not read shadow value!");

    if (g_depth.GetIntProperty ("NoSampleValue", no_sample_value) != XN_STATUS_OK)
      printf ("[OpenNIDriver] Could not read no sample value!");

    return nRetVal;
}


// -----------------------------------------------------------------------------------------------------
//  captureRGB
// -----------------------------------------------------------------------------------------------------
void captureRGB(const XnRGB24Pixel* pImageMap, IplImage* tmp_img = 0, bool doSave = false){

    // Convert to IplImage 24 bit, 3 channels
    for(unsigned int i = 0; i < g_imageMD.XRes()*g_imageMD.YRes();i++)
    {
        tmp_img->imageData[3*i+0]=pImageMap[i].nBlue;
        tmp_img->imageData[3*i+1]=pImageMap[i].nGreen;
        tmp_img->imageData[3*i+2]=pImageMap[i].nRed;
    }
    if (doSave){
        char buf[256];
        sprintf(buf, "data/frame%d_rgb.bmp",g_depthMD.FrameID());
        cvSaveImage(buf, tmp_img);
    }
}

// -----------------------------------------------------------------------------------------------------
//  captureDepth
// -----------------------------------------------------------------------------------------------------
int captureDepth(const XnRGB24Pixel* pImageMap, const XnDepthPixel* pDepthMap, IplImage* tmp_depth = 0, bool saveImage=false, bool savePointCloud = false)
{
	// Calculate the accumulative histogram (the yellow display...)
	const XnDepthPixel* pDepth = g_depthMD.Data();    
	xnOSMemSet(g_pDepthHist, 0, MAX_DEPTH*sizeof(float));
	unsigned int nNumberOfPoints = 0;
	// count depth values
	for (XnUInt y = 0; y < g_depthMD.YRes(); ++y)
	{
		for (XnUInt x = 0; x < g_depthMD.XRes(); ++x, ++pDepth)
		{
			if (*pDepth != 0)
			{
				g_pDepthHist[*pDepth]++;
				nNumberOfPoints++;
			}
		}
	}
	// cumulative sum
	for (int nIndex=1; nIndex<MAX_DEPTH; nIndex++)
	{
		g_pDepthHist[nIndex] += g_pDepthHist[nIndex-1];
	}
	// rescale to 0..256
	if (nNumberOfPoints)
	{
		for (int nIndex=1; nIndex<MAX_DEPTH; nIndex++)
		{
			g_pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (g_pDepthHist[nIndex] / nNumberOfPoints)));
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
				nHistValue = g_pDepthHist[*pDepth];
			
			// yellow pixels
			tmp_depth->imageData[3*i+0] = 0;			//Blue
			tmp_depth->imageData[3*i+1] = nHistValue;	//Green
			tmp_depth->imageData[3*i+2] = nHistValue;	//Red
		}
	}
    if (saveImage){   
        char buf2[256];
        sprintf(buf2,"data/frame%d_depth.bmp",g_depthMD.FrameID());
        cvSaveImage(buf2, tmp_depth);
    }
    
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
				pcl::PointXYZRGB& pt = cloud_save(ind_x,ind_y);
		
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
					rgb = (((unsigned int)pImageMap[depth_index].nRed) << 16) | (((unsigned int)pImageMap[depth_index].nGreen) << 8) | ((unsigned int)pImageMap[depth_index].nBlue);
					pt.rgb = *reinterpret_cast<float*>(&rgb);
				}
			}
		}
		
		char buf[256];
		sprintf(buf, "data/cloud%d.pcd", g_depthMD.FrameID());
		pcl::io::savePCDFile(buf, cloud_save, true);
		// bug in PCL - the binary file is not created with the good rights!
		char bufsys[256];
		sprintf(bufsys, "chmod a+r %s", buf);
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
			
			printf("Test: Frame %02d (%dx%d) Middle point is: %u. FPS: %f\n",
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

			captureRGB(pImageMap, save_img_rgb,true);
			int frameID = captureDepth(pImageMap, pDepthMap,save_img_depth,true,true);
			//usleep(100000);
			
			printf("--- Adding frame %d in sequence ---\n", frameID);
			framesID.push_back(frameID);
			
			// allocate a new DepthMetaData instance
			DepthMetaData *pDepthMD = new DepthMetaData;
			// copy the underlying buffer
			pDepthMD->CopyFrom(g_depthMD);
			// store the instance pointer
			g_vectDepthMD.push_back(pDepthMD);
		}
	}
}

// -----------------------------------------------------------------------------------------------------
//  Main computeInliersAndError
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

        if(error > max_inlier_error_in_m) 
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
void matchFrames(IplImage* &prev_img, struct feature* &prev_features, int &prev_nbfeatures,
		int frameID1,
		int frameID2,
		DepthMetaData *metaDataFrame1,
		DepthMetaData *metaDataFrame2)
{
    Timer tm;

	//char buf1[256], buf2[256];
	//sprintf(buf1, "data/cloudXYZ%d.pcd", frameID1);
	//sprintf(buf2, "data/cloudXYZ%d.pcd", frameID2);
	// generate range image
	char buf2[256];
	/*
	sprintf(buf1, "data/cloudXYZ%d.pcd", frameID1);
	sprintf(buf2, "data/cloudXYZ%d.pcd", frameID2);            
	visualizeRangeImage2(buf1,buf2);*/
	
	// OpenCV image type
	IplImage *img2 = NULL;
	struct feature *features2=NULL; // SIFT library keypoint type
	int n2=0;
	
	IplImage* imgStacked = NULL;
	
	CvFont font;
	double hScale=0.5;
	double vScale=0.5;
	int    lineWidth=1;
	cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, hScale,vScale,0,lineWidth);


	// ---------------------------------------------------------------------------
	// feature detection
	// ---------------------------------------------------------------------------
    tm.start();
	printf("Frames %03d-%03d:\t Extracting SIFT features... ", frameID1, frameID2);
	fflush(stdout);
	
	// load image using OpenCV and detect keypoints in previous frame
	if (prev_features == NULL)
	{
		printf("With first frame... ");		
		char buf[256];
		sprintf(buf, "data/frame%d_rgb.bmp", frameID1);            
		prev_img = cvLoadImage( buf, 1 );
		if (prev_img != NULL)
			prev_nbfeatures = sift_features( prev_img, &prev_features );
		
		// draw SIFT features 
		draw_features(prev_img, prev_features, prev_nbfeatures);
		cvPutText(prev_img, buf, cvPoint(5, 20), &font, cvScalar(255,255,0));
	}
	
	// load image using OpenCV and detect keypoints in new frame
	sprintf(buf2, "data/frame%d_rgb.bmp", frameID2);            
	img2 = cvLoadImage( buf2, 1 );
	if (img2 != NULL)
		n2 = sift_features( img2, &features2 );
	
	// draw SIFT features  
	draw_features(img2, features2, n2);
	cvPutText(img2, buf2, cvPoint(5, 20), &font, cvScalar(255,255,0));	

	// stack the 2 images
	imgStacked = stack_imgs( prev_img, img2 );
				
    tm.stop();
	printf("\t%d + %d features.\t(%dms)\n", prev_nbfeatures, n2, tm.duration());
	fflush(stdout);
	
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
    
	float constant = 0.001 / rgb_focal_length_VGA;    // TODO - redefine this properly
	
	const XnDepthPixel* pDepthData1 = metaDataFrame1->Data();
	const XnDepthPixel* pDepthData2 = metaDataFrame2->Data();
	
	// TODO before K-search clean the features without depth information because they are useless
	//struct feature* featureSearch = NULL;
	//for (int iFeature=0; iFeature<n2; iFeature++)
	//{
	//	if (pDepthData2[cvRound(features2[iFeature]->y) * 640 + cvRound(features2[iFeature]->x)] > 0)
	//}
	
	// try match the new features found in the 2d frame...
	kd_root = kdtree_build( features2, n2 );
	for( i=0; i < prev_nbfeatures; i++ )
	{
		// ... looking in the 1st frame
		feat = prev_features + i;
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
	            const struct feature* feat1 = &prev_features[i];
	            const struct feature* feat2 = neighbour_features[0];
	            
				// draw a line through the 2 points in the stacked image
				pt1 = cvPoint( cvRound( feat1->x ), cvRound( feat1->y ) );
				pt2 = cvPoint( cvRound( feat2->x ), cvRound( feat2->y ) );
				pt2.y += prev_img->height;
				nb_matches++;
				
				// read depth info
				const XnDepthPixel depth1 = pDepthData1[cvRound(feat1->y) * 640 + cvRound(feat1->x)];
				const XnDepthPixel depth2 = pDepthData2[cvRound(feat2->y) * 640 + cvRound(feat2->x)];
				
				// check if depth values are close enough (values in mm)
				if (depth1>0 && //depth1<2000 &&
					depth2>0 && //depth2<2000 &&
					abs(depth1-depth2)/float(depth1) < 0.05)	// read: relative diff 
				{
					// draw a green line
					cvLine( imgStacked, pt1, pt2, CV_RGB(0,255,0), 1, 8, 0 );
					// this is a valid match
					nb_valid_matches++;
					// fwd link the previous features to the new features according to the match
					prev_features[i].fwd_match = neighbour_features[0];
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
	char buf[256];
	sprintf(buf, "data/sift_stacked%d.bmp", frameID1);
	cvSaveImage(buf, imgStacked);

	tm.stop();
    fprintf( stderr, "\tKeeping %d/%d matches.\t(%dms)\n", nb_valid_matches, nb_matches, tm.duration() );
	fflush(stdout);
	//display_big_img( imgStacked, "Matches" );

	/*
	// ---------------------------------------------------------------------------
	// compute a RANSAC transformation (planar)
	// ---------------------------------------------------------------------------
	if (nb_valid_matches>0)
	{
		CvMat* H = NULL;
		IplImage* xformed = NULL;
		
		tm.start();
	    fprintf( stderr, "Frames %03d-%03d:\t RANSAC Transform... ", frameID1, frameID2 );			
		fflush(stdout);

	    // transform
		// lsq_homog -> least-squares planar homography
		H = ransac_xform( prev_features, prev_nbfeatures, FEATURE_FWD_MATCH,
				lsq_homog, 4, 0.01, homog_xfer_err, 3.0, NULL, NULL );
		if( H != NULL )
		{
			xformed = cvCreateImage( cvGetSize( img2 ), IPL_DEPTH_8U, 3 );
			cvWarpPerspective( prev_img, xformed, H, 
						CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS,
						cvScalarAll( 0 ) );
			
			char buf[256];
			sprintf(buf, "data/sift_formed%d.bmp", frameID1);
			cvSaveImage(buf, xformed);
			//cvNamedWindow( "Xformed", 1 );
			//cvShowImage( "Xformed", xformed );
			//cvWaitKey( 0 );
			cvReleaseImage( &xformed );
		}
		tm.stop();
	    fprintf( stderr, "done.\t(%dms)\n", tm.duration() );			
		fflush(stdout);

		// print and free the H matrix
		if (H != NULL)
		{
			fprintf(stderr, "Frames %03d-%03d:\t H Matrix (%dx%d): [", frameID1, frameID2, H->rows, H->cols);
			for (int i=0; i<H->rows; i++)
			{
				for (int j=0; j<H->cols; j++)
					fprintf(stderr, "%lf ", H->data.db[i*H->cols + j]);
				fprintf(stderr, "; ");
			}
			fprintf(stderr, "]\n");
			cvReleaseMat( &H );
		}
	 }
	 */
		
	/* --- initial version ---
	/if (nb_valid_matches>0)
	{
	     float constant = 0.001 / rgb_focal_length_VGA;    
		
		// find transform pairs
	    pcl::TransformationFromCorrespondences tfc;
		for (int i=0; i<prev_nbfeatures ; i++)
		{
			fprintf(stderr, "."); fflush(stderr);
			if (prev_features[i].fwd_match != NULL)
			{
				// read depth info
				const XnDepthPixel* p1 = g_vectDepthMD[frameID1-1]->Data();
				const XnDepthPixel* p2 = g_vectDepthMD[frameID2-1]->Data();
				
				const XnDepthPixel d1 = p1[cvRound(feat->y)*640 + cvRound(feat->x)];
				const XnDepthPixel d2 = p2[cvRound(prev_features[i].fwd_match->y)*640 + cvRound(prev_features[i].fwd_match->x)];
				
				// convert pixels to metric
                float x1 = (prev_features[i].x - 320) * d1 * constant;
                float y1 = (prev_features[i].y - 240) * d1 * constant;
                float z1 = d1 * 0.001 ; // because values are in mm
				
                float x2 = (prev_features[i].fwd_match->x - 320) * d2 * constant;
                float y2 = (prev_features[i].fwd_match->y - 240) * d2 * constant;
                float z2 = d2 * 0.001 ; // because values are in mm

				Eigen::Vector3f from(x1,y1,z1);
				Eigen::Vector3f to (x2,y2,z2);
				tfc.add(from, to);
			}
		}
		fprintf(stderr, "\nCompute transfo matrix...\n"); fflush(stderr);
		
		// get relative movement from samples
		Eigen::Matrix4f transformation = tfc.getTransformation().matrix();
		
		// print and free the H matrix
		std::cout << transformation << "\n" << std::endl;
		fflush(stdout);
	}*/
		
	if (nb_valid_matches>0)
	{
		// find transform pairs
	    pcl::TransformationFromCorrespondences tfc;
        Eigen::Matrix4f best_transformation;
        std::vector<int> index_best_inliers;
        double best_error = 0.1;
	    
		for (int iteration=0; iteration<NB_RANSAC_ITERATIONS ; iteration++)
		{
			fprintf(stderr, "\nIteration %d ... \t", iteration+1);
			tfc.reset();
			// pickup 3 points from matches
	        for (int k = 0; k < 3; k++) {
	            int id_match = rand() % nb_valid_matches;
	            //int index1 = index_matches[id_match];
				tfc.add(vector_matches_orig[id_match], vector_matches_dest[id_match]);
	        }
	        // compute transformation from matches
	        Eigen::Matrix4f transformation = tfc.getTransformation().matrix();
	        
	        // compute error and keep only inliers
	        std::vector<int> index_inliers;
	        double mean_error;
	        double max_inlier_distance_in_m = 0.02;
	        
	        computeInliersAndError(transformation,
	        		vector_matches_orig,
	        		vector_matches_dest,
	        		index_inliers,
	        		mean_error,
	        		max_inlier_distance_in_m * max_inlier_distance_in_m);
	        
	        fprintf(stderr, "Found %d inliers and error:%f", index_inliers.size(), mean_error);
	        
	        if (mean_error<0 || mean_error >= max_inlier_distance_in_m)
	        	continue;	// skip these 3 points and go for a new iteration
	        	
	        if (index_inliers.size()<MIN_NB_INLIERS)
	        	continue;	// not enough inliers found
	        
			if (mean_error < best_error)
	        {
		        fprintf(stderr, "\t => Best candidate transformation! ", index_inliers.size(), mean_error);
	        	best_transformation = transformation;
	        	best_error = mean_error;
	        	index_best_inliers = index_inliers;
	        }
	        
			// ----------------------------------------------------
			// recompute a new transformation with the inliers
			// ----------------------------------------------------
			fprintf(stderr, "\nRecomputing transfo... \t");
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
				
			if (index_inliers.size()<MIN_NB_INLIERS)
				continue;	// not enough inliers found
			
			fprintf(stderr, "Found %d inliers and error:%f", index_inliers.size(), mean_error);
			
			if (mean_error < best_error)
			{
				fprintf(stderr, "\t => Best transformation! ", index_inliers.size(), mean_error);
				best_transformation = transformation;
				best_error = mean_error;
				index_best_inliers = index_inliers;
			}
		}
		fprintf(stderr, "\n");
		
		
		if (index_best_inliers.size()>0)
		{
			// print the transformation matrix
			std::cerr << "Best Transformation --->\t" << index_best_inliers.size() << " inliers and mean error="<< best_error << std::endl;
			std:cerr << best_transformation << std::endl;
			fflush(stderr);

			PoseTransformation pose;
			pose._is_valid = true;
			pose._matrix = best_transformation;
			pose._error = best_error;
			g_transformations.push_back(pose);
			
			// draw inliers
			IplImage* stacked_inliers = NULL;
			
			// stack the 2 images
			stacked_inliers = stack_imgs( prev_img, img2 );
						
			for (int i=0; i<index_best_inliers.size(); i++)
			{
				int id_inlier = index_best_inliers[i];
				int id_match = index_matches[id_inlier];
	            const struct feature* feat1 = &prev_features[id_match];
	            const struct feature* feat2 = prev_features[id_match].fwd_match;
				
				// draw a line through the 2 points in the stacked image
				pt1 = cvPoint( cvRound( feat1->x ), cvRound( feat1->y ) );
				pt2 = cvPoint( cvRound( feat2->x ), cvRound( feat2->y ) );
				pt2.y += prev_img->height;
				// draw a green line
				cvLine( stacked_inliers, pt1, pt2, CV_RGB(0,255,0), 1, 8, 0 );
			}
			fprintf(stderr, "\n"); fflush(stderr);
			
			// save stacked image
			char buf[256];
			sprintf(buf, "data/sift_stacked%d_inliers.bmp", frameID1);
			cvSaveImage(buf, stacked_inliers);
			cvReleaseImage(&stacked_inliers);
		}
		else
		{
			fprintf(stderr, "No transformation found!\n");
			//Eigen::Matrix4f tfo = Eigen::Matrix4f::Identity();
			//g_transformations.push_back(tfo);
			
			PoseTransformation pose;
			pose._is_valid = false;
			pose._matrix = Eigen::Matrix4f::Identity();
			pose._error = 1.0;
			g_transformations.push_back(pose);
									
		}
	}
	
	// free data
	cvReleaseImage(&prev_img);
	free(prev_features);
	
	// assign previous data to the last frame
	prev_img = img2;
	prev_features = features2;
	prev_nbfeatures = n2;
		
	cvReleaseImage(&imgStacked);
}

// -----------------------------------------------------------------------------------------------------
//  buildMap
// -----------------------------------------------------------------------------------------------------
void buildMap(vector<int> &framesID, bool savePointCloud)
{
	char buf_full[256];
	sprintf(buf_full, "data/cloud_full.pcd");
	pcl::PointCloud<pcl::PointXYZRGB> cloudFull;
	pcl::PointCloud<pcl::PointXYZRGB> cloudFrame;
	pcl::PointCloud<pcl::PointXYZRGB> cloudFrameTransformed;
	
	Eigen::Vector4f cameraPose(0, 0, 0, 1.0); 
	Eigen::Matrix4f cumulatedTransformation = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f inverseTfo;
	bool valid_sequence = true;
	
	if (savePointCloud && g_transformations.size()>0)
	{
		cout << "Initialize point cloud frame " << framesID[0] << " (#1/" << framesID.size() << ")..." << std::endl;
		char buf[256];
		sprintf(buf, "data/cloud%d.pcd", framesID[0]);
		pcl::io::loadPCDFile(buf, cloudFull);
	}
	
	for (int iPose=0; iPose<g_transformations.size(); iPose++)
	{
		cout << "----------------------------------------------------------------------------" << std::endl;
		pcl::PointXYZRGB ptCameraPose;
		cameraPose = g_transformations[iPose]._matrix.inverse() * cameraPose;
		cout << cameraPose << std::endl;
		//ptCameraPose.x = cameraPose[0];
		//ptCameraPose.y = cameraPose[1];
		//ptCameraPose.z = cameraPose[2];
		//ptCameraPose.rgb = 255;
		
		// compute global transformation from the start
		cumulatedTransformation = cumulatedTransformation * g_transformations[iPose]._matrix;

		cout << "Mean error:" << g_transformations[iPose]._error << std::endl;
		if (! g_transformations[iPose]._is_valid)
		{
			valid_sequence = false;
			cout << "--- Invalid sequence - aborting point cloud accumulation --- \n" << std::endl;
		}
		
		// update global point cloud
		if (savePointCloud && valid_sequence)
		{
			cout << "Generating point cloud frame " << framesID[iPose+1] << " (#" << iPose+2 << "/" << framesID.size() << ")...";
			char buf[256];
			sprintf(buf, "data/cloud%d.pcd", framesID[iPose+1]);
			pcl::io::loadPCDFile(buf, cloudFrame);

			/*// correct axis orientation
			for (int ind_y =0; ind_y < g_depthMD.YRes(); ind_y++)
			{
				for (int ind_x=0; ind_x < g_depthMD.XRes(); ind_x++)
				{
					pcl::PointXYZRGB& pt = cloud_frame(ind_x,ind_y);
					pt.x = -pt.x;
					pt.z = -pt.z;
				}
			}*/
			
			// inverse transform
			inverseTfo = cumulatedTransformation.inverse();
			// apply transformation to the point cloud
			pcl::getTransformedPointCloud(
					cloudFrame,
					Eigen::Affine3f(inverseTfo),
					cloudFrameTransformed); 
			
			// apend transformed point cloud
			cloudFull += cloudFrameTransformed;
			cout << " +" << cloudFrame.size() << " points => " << cloudFull.size() << " total." << std::endl;
			
		}
	}
	if (savePointCloud && cloudFull.size()>0)
	{
		//cout << "Saving global point cloud ASCII..." << std::endl;
		//pcl::io::savePCDFile(buf_full, cloud_full);
		cout << "Saving global point cloud binary..." << std::endl;    			
		sprintf(buf_full, "data/cloud_full.pcd");
		pcl::io::savePCDFile(buf_full, cloudFull, true);
		// bug in PCL - the binary file is not created with the good rights!
		system("chmod a+r data/cloud_full.pcd");
	}
}

// -----------------------------------------------------------------------------------------------------
//  Main program
// -----------------------------------------------------------------------------------------------------
int main(int argc, char** argv)
{
	if (argc<1)
	{
		printf("Usage: %s --save nbFrames", argv[0]);
		return -1;
	}

    printf("Start\n");
    if (strcmp(argv[1], "--save") == 0)
    {
        XnStatus nRetVal = XN_STATUS_OK;
    	bool savePointCloud = true;
    	int nbFrames=2;	// by default
    	
    	if (argc>2)
    		nbFrames = atoi(argv[2]);
    	
    	if (nbFrames<2)
    	{
    		printf("At least 2 frames are required!\n");    		
			return -1;
    	}
    		
    	if (argc>3 && atoi(argv[3])<=0)
    		savePointCloud = false;
    	
        boost::filesystem::create_directories(saveFolderName);       

        nRetVal = connectKinect();
        if (nRetVal == XN_STATUS_OK)
        {
        	vector<int> sequenceFramesID;
            
        	cloud_save.width = 640;
            cloud_save.height = 480;
            cloud_save.points.resize (640*480);
        	
        	// generate n frames and get its sequence
        	generateFrames(nbFrames, sequenceFramesID);
        	
        	if (sequenceFramesID.size()>=2)
        	{
        		// store the previous image and feature
    			IplImage *pImage = NULL;	    		// OpenCV image type
    			struct feature *pFeatures=NULL;	// SIFT library keypoint type
    			int pNb=0;
    			
        		for (int iFrame=1; iFrame<sequenceFramesID.size(); iFrame++)
        		{
            		// match frame to frame (current with previous)
        			matchFrames(pImage, pFeatures, pNb, sequenceFramesID[iFrame-1], sequenceFramesID[iFrame], g_vectDepthMD[iFrame-1], g_vectDepthMD[iFrame]);
        		}
        		
        		// build map, generates point cloud
        		buildMap(sequenceFramesID, savePointCloud);
        		
        		// free data
        		if (pImage!= NULL)
        			cvReleaseImage(&pImage);
        		if (pFeatures != NULL)
					free(pFeatures);
        		for (int i=0; i<g_vectDepthMD.size(); i++)
        			delete g_vectDepthMD[i];
        	}
            g_context.Shutdown();
        }
    }
    else {
    	printf("Select a valid option");
    	return -1;
    }
    
    //cvWaitKey(0);
    //cvReleaseImage(&img1);
    //cvReleaseImage(&img2);
    return 0;
}

#include "CameraDevice.h"
#include "Config.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "cv.h"
#include "highgui.h"

// Open NI
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>

using namespace xn;
using namespace std;

#define MAX_DEPTH_HISTOGRAM 10000		// previously used for histogram only

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

// working buffers reused for each frame (just to avoid reallocate the arrays each time) 
IplImage* g_imgRGB = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);
IplImage* g_imgDepth = cvCreateImage(cvSize(640,480),IPL_DEPTH_8U,3);

// PCL
pcl::PointCloud<pcl::PointXYZRGB> g_cloudPointSave;

float CameraDevice::_FocalLength = 525;



// -----------------------------------------------------------------------------------------------------
//  common data types
// -----------------------------------------------------------------------------------------------------
float bad_point = std::numeric_limits<float>::quiet_NaN ();

#define CHECK_RC(rc, what)                                            \
    if (rc != XN_STATUS_OK)                                            \
{                                                                \
	printf("%s failed: %s\n", what, xnGetStatusString(rc));        \
	return rc;                                                    \
}

// -----------------------------------------------------------------------------------------------------
//  connect
// -----------------------------------------------------------------------------------------------------
bool CameraDevice::connect()
{
	//Connect to kinect
	printf("Connecting to Kinect... ");
	fflush(stdout);
	XnStatus nRetVal = XN_STATUS_OK;
	EnumerationErrors errors;
	nRetVal = g_context.InitFromXmlFile(Config::_PathKinectXmlFile.c_str(), &errors);
	if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
	{
		XnChar strError[1024];
		errors.ToString(strError, 1024);
		printf("%s\n", strError);
		return false;
	}
	else if (nRetVal != XN_STATUS_OK)
	{
		printf("Open failed: %s\n", xnGetStatusString(nRetVal));
		return false;
	}
	printf("OK\n");

	// allocate the point cloud buffer
	g_cloudPointSave.width = 640;
    g_cloudPointSave.height = 480;
    g_cloudPointSave.points.resize(640*480);

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

    return (nRetVal == XN_STATUS_OK);
}

// -----------------------------------------------------------------------------------------------------
//  disconnect
// -----------------------------------------------------------------------------------------------------
void CameraDevice::disconnect()
{
	g_context.Shutdown();
}

// -----------------------------------------------------------------------------------------------------
//  saveRGBImage
// -----------------------------------------------------------------------------------------------------
void saveRGBImage(const XnRGB24Pixel* pImageMap, IplImage* tmp_img, int frameID)
{
	if (frameID<0)
		frameID = g_imageMD.FrameID();	// use ID given by Kinect
	
	// Convert to IplImage 24 bit, 3 channels
	for(unsigned int i = 0; i < g_imageMD.XRes()*g_imageMD.YRes();i++)
	{
		tmp_img->imageData[3*i+0]=pImageMap[i].nBlue;
		tmp_img->imageData[3*i+1]=pImageMap[i].nGreen;
		tmp_img->imageData[3*i+2]=pImageMap[i].nRed;
	}

	char buf[256];
	sprintf(buf, "%s/frame_%d_rgb.bmp", Config::_DataDirectory.c_str(), frameID);
	cvSaveImage(buf, tmp_img);
}

// -----------------------------------------------------------------------------------------------------
//  saveHistogramImage
// -----------------------------------------------------------------------------------------------------
int saveHistogramImage(
		const XnRGB24Pixel* pImageMap,
		const XnDepthPixel* pDepthMap,
		IplImage* pImgDepth,
		int frameID)
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

	if (frameID<0)
	frameID = g_depthMD.FrameID();	// use ID given by Kinect

	
	char bufFilename[256];
	sprintf(bufFilename,"%s/frame_%d_histo.bmp", Config::_DataDirectory.c_str(), frameID);
	cvSaveImage(bufFilename, pImgDepth);
}

// -----------------------------------------------------------------------------------------------------
//  saveDepthImage
// -----------------------------------------------------------------------------------------------------
int saveDepthImage(
		const XnRGB24Pixel* pImageMap,
		const XnDepthPixel* pDepthMap,
		IplImage* pImgDepth,
		int frameID,
		bool savePointCloud)
{
	if (frameID<0)
		frameID = g_depthMD.FrameID();	// use ID given by Kinect
	
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
	sprintf(bufFilename, "%s/frame_%d_depth.bmp", Config::_DataDirectory.c_str(), frameID);
	cvSaveImage(bufFilename, pImgDepth);
    
	// point cloud
    if (savePointCloud)
    {
		float constant = 0.001 / CameraDevice::_FocalLength;    
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
		sprintf(buf, "%s/cloud%d.pcd", Config::_ResultDirectory.c_str(), frameID);
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
bool CameraDevice::generateFrame(int &frameID)
{
    XnStatus nRetVal = XN_STATUS_OK;
	
	const XnDepthPixel* pDepthMap = NULL;
	const XnRGB24Pixel* pImageMap = NULL;
	
	while(true)
	{
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
			char c = xnOSReadCharFromInput();	// reset the keyboard hit
			printf("\n");
			if (c == 27)	// ESC
				break;

			saveRGBImage(pImageMap, g_imgRGB, frameID);
			int kinectFrameID = saveDepthImage(pImageMap, pDepthMap, g_imgDepth, frameID, false);	// no PCD file
			//usleep(100000);
			printf("Adding frame %d, saved as id=%d\n", kinectFrameID, frameID);
			return true;
		}
	}
	return false;
}

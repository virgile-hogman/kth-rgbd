///////////////////
// SIFT EXTRACTION
///////////////////


#include "FrameData.h"

// Open CV
//#include "cxcore.h"
//#include "extract.h"

// standard
#include <stdio.h>

// -----------------------------------------------------------------------------------------------------
//  FrameData
// -----------------------------------------------------------------------------------------------------
FrameData::FrameData()
{
	_frameID = -1;
	_pImage = NULL;
	_pFeatures = NULL;
	_nbFeatures = 0;
	_depthData = NULL;
}

FrameData::~FrameData()
{
	// free memory
	releaseData();
}

bool FrameData::loadImage(int frameID)
{
	char buf[256];
	
	printf("Loading RGB data for frame %d\n", frameID);
	fflush(stdout);
	
	sprintf(buf, "data/frame%d_rgb.bmp", frameID);            
	_pImage  = cvLoadImage( buf, 1 );
	if (_pImage != NULL)
		_frameID = frameID;
	else
		_frameID = -1;
	return (_pImage != NULL);
}

bool FrameData::isLoaded(int frameID)
{
	return (_frameID == frameID && _pImage != NULL);
}

bool FrameData::loadDepthData()
{
	char buf[256];
	IplImage *pImageDepth = NULL;
	
	printf("Loading depth data for frame %d\n", _frameID);
	fflush(stdout);
	
	if (_depthData != NULL)
		delete _depthData;
	_depthData = NULL;
	
	sprintf(buf,"data/frame%d_depth.bmp", _frameID);
	pImageDepth = cvLoadImage( buf, -1 );	// read 1 channel
	if (pImageDepth != NULL)
	{
		printf("Nb Channels: %d depth:%d/%d\n", pImageDepth->nChannels, pImageDepth->depth, IPL_DEPTH_16U);
		fflush(stdout);
		
		pImageDepth->depth = IPL_DEPTH_16U;		
		
		// allocate depth buffer
		_depthData = new TDepthPixel[640*480];
		for(int i = 0; i < 640*480;i++)
		{
			TDepthPixel depthByte1 = (unsigned char)(pImageDepth->imageData[3*i+0]);
			TDepthPixel depthByte2 = (unsigned char)(pImageDepth->imageData[3*i+1]);
			// depth pixels on 16 bits
			_depthData[i] = (depthByte1<<8) | depthByte2;
		}
		printf("Depth value reloaded at (320,240):%x\n", _depthData[640*240 + 320]);				
	}
	return (_depthData != NULL);
}

void FrameData::releaseData()
{
	// free memory
	if (_pImage != NULL)
		cvReleaseImage(&_pImage);
	if (_pFeatures != NULL)		
		free(_pFeatures);
	if (_depthData != NULL)
		delete _depthData;

	_frameID = -1;
	_pImage = NULL;
	_pFeatures = NULL;
	_nbFeatures = 0;
	_depthData = NULL;
}

void FrameData::assignData(FrameData &srcFrameData)
{
	releaseData();
	// recopy pointers only, data is not reallocated
	_frameID = srcFrameData._frameID;
	_pImage = srcFrameData._pImage;
	_pFeatures = srcFrameData._pFeatures;
	_nbFeatures = srcFrameData._nbFeatures;
	_depthData = srcFrameData._depthData;
	// all the data is now handled by the new object so the source loses it
	srcFrameData._frameID = -1;
	srcFrameData._pImage = NULL;
	srcFrameData._pFeatures = NULL;
	srcFrameData._nbFeatures = 0;
	srcFrameData._depthData = NULL;
}

int FrameData::computeFeatures()
{	
	if (_pImage != NULL)
		_nbFeatures = sift_features( _pImage, &_pFeatures );
	else
		_nbFeatures = 0;
	return _nbFeatures;
}

void FrameData::drawFeatures(CvFont &font)
{
	char buf[256];
	sprintf(buf,"Frame%d", _frameID);
	// draw SIFT features 
	draw_features(_pImage, _pFeatures, _nbFeatures);
	cvPutText(_pImage, buf, cvPoint(5, 20), &font, cvScalar(255,255,0));
}




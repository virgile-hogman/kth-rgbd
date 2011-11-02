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
#include "Config.h"

// Open CV
#include "opencv/highgui.h"
#include <ctype.h>

// standard
#include <vector>
#include <list>
#include <stdio.h>

using namespace std;

string FrameData::_DataPath;

const char *strFileExtension = "bmp";
const char *strSearchExtension = ".bmp";

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
	_featureType = FEATURE_SURF;
}

FrameData::~FrameData()
{
	// free memory
	releaseData();
}

bool FrameData::loadImageRGBD(int frameID)
{
	_frameID = frameID;	// valid ID
	if (loadImageRGB() && loadImageDepth())
		return true;

	_frameID = -1;		// invalid ID
	return false;
}

bool FrameData::loadImageRGB()
{
	char filename[256];
	
	if (_pImage != NULL) {
		cvReleaseImage(&_pImage);
		_pImage = NULL;
	}

	// load RGB data file
	sprintf(filename, "%s/frame_%d_rgb.%s", _DataPath.c_str(), _frameID, strFileExtension);
	_pImage  = cvLoadImage(filename, 1);
	
	if (_pImage == NULL)
	{
		string pathFile;
		sprintf(filename, "frame_%d_rgb", _frameID);
		if (FindFrame(_DataPath.c_str(), filename,  pathFile))
			_pImage  = cvLoadImage( pathFile.c_str(), 1 );
	}
	
	return (_pImage != NULL);
}


bool FrameData::loadImageDepth()
{
	char filename[256];
	IplImage *pImageDepth = NULL;
	
	if (_depthData != NULL)
		delete[] _depthData;
	_depthData = NULL;

	// load depth data file, ID must previously be set
	sprintf(filename,"%s/frame_%d_depth.%s", _DataPath.c_str(), _frameID, strFileExtension);
	pImageDepth = cvLoadImage(filename, -1);	// read channels as defined in file
	
	if (pImageDepth == NULL)
	{
		string pathFile;
		sprintf(filename, "frame_%d_depth", _frameID);
		if (FindFrame(_DataPath.c_str(), filename,  pathFile))
			pImageDepth  = cvLoadImage( pathFile.c_str(), -1 );
	}
	
	if (pImageDepth != NULL)
	{
		// allocate depth buffer
		if (_depthData == NULL)
			_depthData = new TDepthPixel[pImageDepth->width * pImageDepth->height];
		// otherwise assume it has the correct size (only 1 format handled at a time)
		
		for(int i = 0; i < pImageDepth->width * pImageDepth->height; i++)
		{
			// depth pixels on 16 bits but it is splitted on 2 channels * 8U in the file
			TDepthPixel depthByte1 = (unsigned char)(pImageDepth->imageData[3*i+0]);
			TDepthPixel depthByte2 = (unsigned char)(pImageDepth->imageData[3*i+1]);
			_depthData[i] = (depthByte1<<8) | depthByte2;
		}
		//printf("Depth value reloaded at (320,240):%x\n", _depthData[640*240 + 320]);
		cvReleaseImage(&pImageDepth);
	}
	else
	{
		// free depth buffer
		if (_depthData != NULL)
			delete _depthData;
		_depthData = NULL;
	}
	
	return (_depthData != NULL);
}

void FrameData::saveImageRGB(const char *path)
{
	char filename[256];
	if (_pImage != NULL) {
		if (path == NULL)
			path = Config::_PathFrameSequence.c_str();
		sprintf(filename, "%s/frame_%d_rgb.%s", path, _frameID, strFileExtension);
		cvSaveImage(filename, _pImage);
		printf("Generated file: %s\n", filename);
	}
}

bool FrameData::createImageRGB()
{
	// here we assume size is constant, we reaffect existing buffer
	if (_pImage == NULL)
		_pImage = cvCreateImage(cvSize(NBPIXELS_WIDTH, NBPIXELS_HEIGHT),IPL_DEPTH_8U,3);
	return (_pImage != NULL);
}

void FrameData::releaseImageRGB()
{
	if (_pImage != NULL) {
		cvReleaseImage(&_pImage);
		_pImage = NULL;
	}
}

void FrameData::copyImageRGB(IplImage *pImageRGB)
{
	if (_pImage != NULL) {
		cvReleaseImage(&_pImage);
		_pImage = NULL;
	}

	if (pImageRGB != NULL)
		_pImage=cvCloneImage(pImageRGB);
}

void FrameData::copyImageDepth(IplImage *pImageDepth)
{
	if (_depthData != NULL)
		delete[] _depthData;
	_depthData = NULL;

	if (pImageDepth != NULL)
	{
		_depthData = new TDepthPixel[pImageDepth->width * pImageDepth->height];
		for(int i = 0; i < pImageDepth->width * pImageDepth->height; i++)
		{
			// depth pixels on 16 bits but it is splitted on 2 channels * 8U in the file
			TDepthPixel depthByte1 = (unsigned char)(pImageDepth->imageData[3*i+0]);
			TDepthPixel depthByte2 = (unsigned char)(pImageDepth->imageData[3*i+1]);
			_depthData[i] = (depthByte1<<8) | depthByte2;
		}
	}
}

bool FrameData::fetchFeatures(int frameID)
{
	// loads data and compute features only if necessary
	if (_frameID != frameID)
	{
		if (!loadImageRGBD(frameID))
			return false;

		computeFeatures();
		drawFeatures();
	}
	return true;
}

void FrameData::releaseImageRGBD()
{
	if (_pImage != NULL)
		cvReleaseImage(&_pImage);
	_pImage = NULL;
	if (_depthData != NULL)
		delete[] _depthData;
	_depthData = NULL;
}

void FrameData::releaseFeatures()
{
	if (_pFeatures != NULL)
	{
		// free user data
		for (int i=0; i<_nbFeatures; i++)
			if (_pFeatures[i].feature_data != NULL)
				delete (TDepthPixel*)(_pFeatures[i].feature_data);
		// free buffer
		free(_pFeatures);
	}
	_pFeatures = NULL;
	_nbFeatures = 0;
}

void FrameData::releaseData()
{
	// free memory
	releaseFeatures();
	releaseImageRGBD();
	_frameID = -1;
	_nbFeatures = 0;
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

void FrameData::copyData(const FrameData &srcFrameData)
{
	releaseData();
	
	// reallocate/clone data
	_frameID = srcFrameData._frameID;
	
	if (srcFrameData._pImage != NULL)
		_pImage=cvCloneImage(srcFrameData._pImage);
	
	_pFeatures = (struct feature*)calloc( _nbFeatures, sizeof(struct feature) );
	memcpy(&_pFeatures, &srcFrameData._pFeatures, sizeof(struct feature) * srcFrameData._nbFeatures);
	
	_nbFeatures = srcFrameData._nbFeatures;

	if (_pImage != NULL && srcFrameData._depthData != NULL)
	{
		_depthData = new TDepthPixel[_pImage->width * _pImage->height];
		memcpy(_depthData, &srcFrameData._depthData, sizeof(TDepthPixel) * _pImage->width * _pImage->height);
	}
}

int FrameData::computeFeatures()
{
	_featureType = (FeatureType)Config::_FeatureType;

	switch(_featureType) {
	case FEATURE_SIFT:
		computeFeaturesSIFT();
		break;
	case FEATURE_SURF:
		computeFeaturesSURF();
		break;
	default:
		// invalid type
		break;
	}

	removeInvalidFeatures();

	// features depth
	if (_nbFeatures>0) {
		for (int i=0; i<_nbFeatures; i++) {
			// compute and store feature depth
			TDepthPixel depth=getDepthPixel(_pFeatures[i].x, _pFeatures[i].y);
			_pFeatures[i].feature_data = new TDepthPixel(depth);
		}
	}
}

int FrameData::computeFeaturesSIFT()
{
	// free the previous buffer
	releaseFeatures();

	// compute the new SIFT features
	if (_pImage != NULL)
		_nbFeatures = sift_features( _pImage, &_pFeatures );

	return _nbFeatures;
}


int FrameData::computeFeaturesSURF()
{
	// free the previous buffer
	releaseFeatures();

	if (_pImage != NULL) {
		IplImage *pImgGray=cvCreateImage(cvSize(_pImage->width, _pImage->height),IPL_DEPTH_8U,1);
		if (pImgGray == NULL)
			return -1;

		CvMemStorage* storage = cvCreateMemStorage(0);
		CvSeq *objectKeypoints = 0, *objectDescriptors = 0;

		// extended descriptor = 128 elements as SIFT default
		CvSURFParams params = cvSURFParams(500,1);
		/*params.nOctaves=6;
		params.nOctaveLayers=4;
		params.hessianThreshold=300;*/

		// convert to grayscale (opencv SURF only works with this format)
		cvCvtColor(_pImage, pImgGray, CV_RGB2GRAY);

		fflush(stdout);
		cvExtractSURF(pImgGray, NULL, &objectKeypoints, &objectDescriptors, storage, params, 0);
		cvReleaseImage(&pImgGray);

		_nbFeatures = objectDescriptors->total;
		fflush(stdout);

		// --------------------------------------------------------------
		// conversion to SIFT structure to reuse the same matching code
		// TODO - remove this and write a more general kNN function
		// --------------------------------------------------------------
	    CvSeqReader reader, kreader;
		cvStartReadSeq( objectKeypoints, &kreader );
		cvStartReadSeq( objectDescriptors, &reader );

		// sort features by decreasing scale and move from CvSeq to array
		//cvSeqSort( features, (CvCmpFunc)feature_cmp, NULL );
		_pFeatures = (feature*)calloc( _nbFeatures, sizeof(struct feature) );
		//feat = cvCvtSeqToArray( features, *feat, CV_WHOLE_SEQ );
		for (int i=0; i<_nbFeatures; i++) {

			const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
			const float* descriptor = (const float*)reader.ptr;
			CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
			CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );

			// coordinates
			_pFeatures[i].x = kp->pt.x;
			_pFeatures[i].y = kp->pt.y;
			// scale
			_pFeatures[i].scl = kp->size*1.2f/9.0f;	// see surf.cpp
			// direction
			_pFeatures[i].ori = kp->dir*PI180;  // SURF in degrees, SIFT in radians
			// descriptor length
			_pFeatures[i].d = 128;
			// descriptor values
			for (int desc=0; desc<128; desc++)
				_pFeatures[i].descr[desc] = descriptor[desc];
			// type
			_pFeatures[i].type = FEATURE_LOWE;
			// category
			_pFeatures[i].category = 0;
			// matching features
			_pFeatures[i].fwd_match = NULL;
			_pFeatures[i].bck_match = NULL;
			_pFeatures[i].mdl_match = NULL;
			// location in image
			_pFeatures[i].img_pt.x = kp->pt.x;
			_pFeatures[i].img_pt.y = kp->pt.y;
			// user data
			_pFeatures[i].feature_data = NULL;
		}
		// -------------------------------------------------------------*/

		cvRelease((void **)&objectKeypoints);
		cvRelease((void **)&objectDescriptors);
		cvReleaseMemStorage(&storage);
	}
	return _nbFeatures;
}

void FrameData::drawFeatures()
{
	CvFont font;
	double hScale=0.5;
	double vScale=0.5;
	int    lineWidth=1;
	char buf[256];

	if (_nbFeatures>0) {
		// use SIFT library
		draw_features(_pImage, _pFeatures, _nbFeatures);
	}

	sprintf(buf,"Frame%d [%d %s]", _frameID, _nbFeatures, (_featureType==FEATURE_SURF)?"SURF":"SIFT");
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale,vScale, 0, lineWidth);
	cvPutText(_pImage, buf, cvPoint(5, 18), &font, cvScalar(255,255,0));
}

void FrameData::removeInvalidFeatures()
{
	vector<int>	validIdFeatures;
	struct feature*		pNewFeatures = NULL; 
	
	const int maxDeltaDepthArea=50;
	const int sizeFeatureArea=-1;

	// generate a list of the valid features
	for (int i=0; i < _nbFeatures; i++)
	{
		TDepthPixel depthFeature = getDepthPixel(_pFeatures[i].x, _pFeatures[i].y);
		if (depthFeature == 0 ||	// no available depth information
			depthFeature < Config::_FeatureDepthMin ||
			depthFeature > Config::_FeatureDepthMax)
			continue;
		
		if (sizeFeatureArea<=0)
			validIdFeatures.push_back(i);
		else
		{
			bool validArea = true;
			// look n pixels around
			for (int row=-sizeFeatureArea; row<=sizeFeatureArea && validArea; row++)
				for (int col=-sizeFeatureArea; col<=sizeFeatureArea && validArea; col++)
				{
					TDepthPixel depthNeighbour = getDepthPixel(_pFeatures[i].x+row, _pFeatures[i].y+col);
					if (depthNeighbour!=0 && abs(depthFeature-depthNeighbour)>maxDeltaDepthArea)
						validArea = false;
				}
			
			if (validArea)
				validIdFeatures.push_back(i);
		}
	}

	if (validIdFeatures.size() == 0) {
		// free the previous buffer
		releaseFeatures();
	}
	else if (validIdFeatures.size() < _nbFeatures)	{
		//printf("Features valid: %d/%d\n", validIdFeatures.size(), _nbFeatures);
		//fflush(stdout);
		
		// allocate a new buffer
		pNewFeatures = (struct feature*)calloc( validIdFeatures.size(), sizeof(struct feature) );
		for(int i=0; i < validIdFeatures.size(); i++ )
		{
			// copy valid feature to new buffer
			memcpy(&pNewFeatures[i], &_pFeatures[validIdFeatures[i]], sizeof(struct feature));
		    // the user data should not be cleared afterwards, it has just been reassigned
			_pFeatures[validIdFeatures[i]].feature_data = NULL;
		}
		
		releaseFeatures();
		_pFeatures = pNewFeatures;
		_nbFeatures = validIdFeatures.size();
	}
}

// -----------------------------------------------------------------------------------------------------
//  FindFrame
// -----------------------------------------------------------------------------------------------------
bool FrameData::FindFrame(
		const boost::filesystem::path & pathSearch,	// in this directory,
        const string & filename,				// search for this name,
        string & filePathFound)				// placing path here if found
{
	int timestamp = 0;
	if (! boost::filesystem::exists(pathSearch))
		return false;

	// looking for pattern <filename>_<timestamp>.<ext>
	char filePattern[256];
	sprintf(filePattern, "%s_%%d", filename.c_str());
	fflush(stdout);

	boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
	for (boost::filesystem::directory_iterator itr( pathSearch ); itr != end_itr; ++itr )
	{
		if (boost::filesystem::extension(*itr)==strSearchExtension &&
			sscanf(itr->leaf().c_str(), filePattern, &timestamp)==1)
		{
			filePathFound = itr->path().string().c_str();
			return true;
		}
	}
	return false;
}

// -----------------------------------------------------------------------------------------------------
//  getFrameList
// -----------------------------------------------------------------------------------------------------
void FrameData::GetFrameList(const char *dataDirectory, int min, int max, vector<int> &sequenceFramesID)
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
		if (boost::filesystem::extension(*itr)==strSearchExtension &&
			sscanf(itr->leaf().c_str(), "frame_%d", &frameID)==1 && frameID>=min && (frameID<=max || max<0))
		{
			// add frame
			//printf("Add frame file #%i:\t%s\n", frameID, itr->path().string().c_str());
			listFramesID.push_back(frameID);
		}
	}
	// sort and keep only 1 element (remove duplicates because of rgb+depth)
	listFramesID.sort();
	listFramesID.unique();

	// build the sequence
	while (!listFramesID.empty())
	{
		sequenceFramesID.push_back(listFramesID.front());
		listFramesID.pop_front();
	}

	cout << "Sequence of " << sequenceFramesID.size() << " frames available.\n";
	cout << "Sequence of " << sequenceFramesID.size()/Config::_DataInRatioFrame << " frames with ";
	cout << "Frame ratio:" << Config::_DataInRatioFrame << "\n";
}

///////////////////
// SIFT EXTRACTION
///////////////////


#include "FrameData.h"


// Open CV
#include "highgui.h"

// standard
#include <vector>
#include <stdio.h>

std::string FrameData::_DataPath;

bool FrameData::find_file( const boost::filesystem::path & dir_path,     		// in this directory,
                const std::string & file_name,	// search for this name,
                std::string & path_found )        		// placing path here if found
{
  if ( ! boost::filesystem::exists( dir_path ) )
	  return false;
  
  
  char bufPattern[256];
  sprintf(bufPattern, "%s_%%d", file_name.c_str());
  int timestamp;
  //printf("Looking for file with pattern %s.bmp\n", bufPattern);
  fflush(stdout);
	  
  boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
  for (  boost::filesystem::directory_iterator itr( dir_path );
        itr != end_itr;
        ++itr )
  {
	  if (boost::filesystem::extension(*itr)==".bmp" &&
		  sscanf(itr->leaf().c_str(), bufPattern, &timestamp)==1)
		{
			// add frame
		  path_found = itr->path().string().c_str();
		  return true;
		}
  }
  return false;
}

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
	
	//printf("Loading RGB data for frame %d\n", frameID);
	//fflush(stdout);
	
	// load RGB data file
	sprintf(buf, "%s/frame_%d_rgb.bmp", _DataPath.c_str(), frameID);    
	_pImage  = cvLoadImage( buf, 1 );
	
	if (_pImage == NULL)
	{
		std::string pathFile;
		sprintf(buf, "frame_%d_rgb", frameID);   
		if (find_file(_DataPath.c_str(), buf,  pathFile))
			_pImage  = cvLoadImage( pathFile.c_str(), 1 );
	}
	
	if (_pImage != NULL)
		_frameID = frameID;	// valid ID
	else
		_frameID = -1;		// invalid ID
	return (_pImage != NULL);
}

bool FrameData::isLoaded(int frameID)
{
	// check only if RGB data is available here
	return (_frameID == frameID && _pImage != NULL);
}

bool FrameData::loadDepthData()
{
	char buf[256];
	IplImage *pImageDepth = NULL;
	
	//printf("Loading depth data for frame %d\n", _frameID);
	//fflush(stdout);
	
	// load depth data file
	sprintf(buf,"%s/frame_%d_depth.bmp", _DataPath.c_str(), _frameID);
	pImageDepth = cvLoadImage( buf, -1 );	// read channels as defined in file
	
	if (pImageDepth == NULL)
	{
		std::string pathFile;
		sprintf(buf, "frame_%d_depth", _frameID);   
		if (find_file(_DataPath.c_str(), buf,  pathFile))
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
		memcpy(_depthData, &srcFrameData._depthData, sizeof(struct feature) * _pImage->width * _pImage->height);
	}
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
	sprintf(buf,"Frame%d nf:%d", _frameID, _nbFeatures);
	// draw SIFT features 
	draw_features(_pImage, _pFeatures, _nbFeatures);
	cvPutText(_pImage, buf, cvPoint(5, 20), &font, cvScalar(255,255,0));
}

void FrameData::removeInvalidFeatures(int sizeSurfaceArea, int maxDeltaDepthArea)
{
	std::vector<int>	validIdFeatures;
	struct feature*		pNewFeatures = NULL; 
	
	// generate a list of the valid features
	for (int i=0; i < _nbFeatures; i++)
	{
		TDepthPixel depthFeature = getFeatureDepth(&_pFeatures[i]); 
		if (depthFeature == 0)	// no available depth information 
			continue;
		
		if (sizeSurfaceArea<=0)	
			validIdFeatures.push_back(i);
		else
		{
			bool validArea = true;
			// look n pixels around
			for (int row=-sizeSurfaceArea; row<=sizeSurfaceArea && validArea; row++)
				for (int col=-sizeSurfaceArea; col<=sizeSurfaceArea && validArea; col++)
				{
					TDepthPixel depthNeighbour = _depthData[(cvRound(_pFeatures[i].y)+col) * 640 + cvRound(_pFeatures[i].x)+row];
					if (depthNeighbour!=0 && abs(depthFeature-depthNeighbour)>maxDeltaDepthArea)
						validArea = false;
				}
			
			if (validArea)
				validIdFeatures.push_back(i);
		}
	}
	
	if (validIdFeatures.size() < _nbFeatures)
	{
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
		
		// free the previous buffer		
		if (_pFeatures != NULL)		
			free(_pFeatures);
		
		_pFeatures = pNewFeatures;
		_nbFeatures = validIdFeatures.size();
	}
}
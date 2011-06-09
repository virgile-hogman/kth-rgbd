#ifndef FRAMEDATA_H
#define FRAMEDATA_H

#include "CommonTypes.h"

#include "cv.h"

extern "C" {
#include "sift.h"
#include "imgfeatures.h"
}

// -----------------------------------------------------------------------------------------------------
//  FrameData
// -----------------------------------------------------------------------------------------------------
class FrameData
{
private:
	int					_frameID;		// frame identifier
	IplImage*			_pImage;		// RGB data
	TDepthPixel*		_depthData;		// depth data
	struct feature*		_pFeatures;		// SIFT features
	int					_nbFeatures;	// number of features (size of features data)
	
	unsigned char		_width;
	unsigned char		_height;
	
public:
	FrameData();
	~FrameData();
	
	// basic public accessors
	int getFrameID()									{ return _frameID; }
	IplImage* getImage()								{ return _pImage; }
	TDepthPixel* getDepthData()							{ return _depthData; }
	struct feature* getFeatures()						{ return _pFeatures; }
	int getNbFeatures()									{ return _nbFeatures; }
	
	// get feature
	const struct feature* getFeature(int i)				{ return &_pFeatures[i]; }
	
	// feature match
	const struct feature* getFeatureMatch(int i)
	{
		return _pFeatures[i].fwd_match;
	}
	void setFeatureMatch(int i, struct feature* featureMatch)
	{
		_pFeatures[i].fwd_match = featureMatch;
	}
	
	// depth data
	const TDepthPixel getDepth(int x, int y)
	{
		return _depthData[y*640 + x];
	}

	// depth for one feature
	const TDepthPixel getFeatureDepth(const feature *feature)
	{
		return _depthData[cvRound(feature->y) * 640 + cvRound(feature->x)];
	}
	
	bool loadImage(int frameID);
	
	bool isLoaded(int frameID);

	bool loadDepthData();

	void releaseData();

	void assignData(FrameData &srcFrameData);
	
	int computeFeatures();

	void drawFeatures(CvFont &font);
	
	void removeInvalidFeatures(int sizeSurfaceArea, int maxDeltaDepthArea);
	
	// directory where to load/save the data files
	static std::string _DataPath;
};

#endif


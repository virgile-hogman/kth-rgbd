#ifndef FRAMEDATA_H
#define FRAMEDATA_H

#include "CommonTypes.h"

#include "opencv/cv.h"

extern "C" {
#include "sift.h"
#include "imgfeatures.h"
}

#include <boost/filesystem.hpp>

enum TypeFeature
{
	FEATURE_SIFT,
    FEATURE_SURF
};

class FrameData
{
private:
	int					_frameID;		// frame identifier
	IplImage*			_pImage;		// RGB data
	TDepthPixel*		_depthData;		// depth data
	struct feature*		_pFeatures;		// SIFT features
	int					_nbFeatures;	// number of features (size of features data)
	TypeFeature			_typeFeature;	// SIFT or SURF
	
public:
	FrameData();
	~FrameData();
	
	// basic public accessors
	int getFrameID() const								{ return _frameID; }
	IplImage* getImage()								{ return _pImage; }
	TDepthPixel* getDepthData()							{ return _depthData; }
	struct feature* getFeatures()						{ return _pFeatures; }
	int getNbFeatures()	const							{ return _nbFeatures; }
	TypeFeature getTypeFeature() const					{ return _typeFeature; }
	
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
	
	bool isLoaded(int frameID) const;

	bool loadDepthData();

	void releaseData();

	void assignData(FrameData &srcFrameData);
	
	void copyData(const FrameData &srcFrameData);
	
	int computeFeatures();

	void drawFeatures();
	
	void removeInvalidFeatures();
	
	void saveImage();

	// directory where to load/save the data files
	static std::string _DataPath;
	
	
	static bool find_file(
			const boost::filesystem::path & dir_path,	// in this directory,
            const std::string & file_name,				// search for this name,
            std::string & path_found );					// placing path here if found

protected:
	int computeFeaturesSIFT();
	int computeFeaturesSURF();
};

#endif


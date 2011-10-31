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

#ifndef FRAMEDATA_H
#define FRAMEDATA_H

#include "CommonTypes.h"

#include "opencv/cv.h"

extern "C" {
#include "sift.h"
#include "imgfeatures.h"
}

#include <boost/filesystem.hpp>

enum FeatureType
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
	FeatureType			_featureType;	// SIFT or SURF
	
public:
	FrameData();
	~FrameData();
	
	// basic public accessors
	int getFrameID() const								{ return _frameID; }
	IplImage* getImage()								{ return _pImage; }
	TDepthPixel* getDepthData()							{ return _depthData; }
	struct feature* getFeatures()						{ return _pFeatures; }
	int getNbFeatures()	const							{ return _nbFeatures; }
	FeatureType getFeatureType() const					{ return _featureType; }
	
	// get feature by index
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
	
	// depth data at given pixel coordinates
	const TDepthPixel getDepthPixel(int x, int y)
	{
		return _depthData[y*NBPIXELS_WIDTH + x];
	}
	const TDepthPixel getDepthPixel(double x, double y)
	{
		return _depthData[cvRound(y) * NBPIXELS_WIDTH + cvRound(x)];
	}

	// depth for one feature
	const TDepthPixel getFeatureDepth(const feature *feature)
	{
		if (feature->feature_data != NULL)
			return *(TDepthPixel*)(feature->feature_data);
		else
			return 0;
	}
	
	// load RGB data
	bool loadImage(int frameID);
	bool isImageLoaded(int frameID) const;
	// load D data only
	bool loadDepthData();
	// save RGB data (with features drawn)
	void saveImage();

	// copy given RGB image to internal RGB buffer
	void copyImageRGB(IplImage *pImageRGB);
	// copy given depth image to internal depth buffer
	void copyImageDepth(IplImage *pImageDepth);

	// loads RGBD data, computes and draws features only if necessary
	bool fetchFeatures(int frameID);
	// extract features keypoint and descriptors
	int computeFeatures();
	// draw feature on RGB data
	void drawFeatures();

	// free data RGBD
	void releaseImageAndDepth();
	// free data features
	void releaseFeatures();
	// free all
	void releaseData();

	// transfer data to current object, without free
	void assignData(FrameData &srcFrameData);
	// duplicate data buffers into current object
	void copyData(const FrameData &srcFrameData);
	
	// directory where to load/save the data files
	static std::string _DataPath;
	
	// utility function to locate RGBD files with the prefix but unknown timestamp
	static bool FindFile(
			const boost::filesystem::path & pathSearch,	// in this directory,
            const std::string & fileName,				// search for this name,
            std::string & fileFound);					// full path to file if found

protected:
	// SIFT features
	int computeFeaturesSIFT();
	// SURF features
	int computeFeaturesSURF();

	// basic filtering on depth
	void removeInvalidFeatures();
};

#endif


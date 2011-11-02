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

#include "cv.h"
#include "highgui.h"

#include "Config.h"
#include "Display.h"
#include "CommonTypes.h"

#define SCORE_OFFSET_X 10
#define SCORE_OFFSET_Y 68
#define SCORE_WIDTH 280
#define SCORE_HEIGHT 10

#define INFO_POSX 640
#define INFO_POSY 0
#define INFO_WIDTH 350
#define INFO_HEIGHT 80

#define FEAT_POSX 0
#define FEAT_POSY 0


CvScalar getColorScore(float score)
{
	// display score bar
	if (score>0.66)
		return cvScalar(50,255,50);		// dark green
	else if (score>0.33)
		return cvScalar (0,165,255);	// orange
	else
		return cvScalar (0,0,255);		// red
}

Display::Display()
{
	_displayingPreview = false;
	_displayingFeatures = false;
	// create buffers
	_pImgInfo = cvCreateImage(cvSize(INFO_WIDTH,INFO_HEIGHT),IPL_DEPTH_8U,3);
	_pImgFeatures = cvCreateImage(cvSize(NBPIXELS_WIDTH,NBPIXELS_HEIGHT*2),IPL_DEPTH_8U,3);

	_totalScore = 0;
	_totalCount = 0;
	_minScore = 1;
}

Display::~Display()
{
	// release buffers
	if (_pImgInfo != NULL)
		cvReleaseImage(&_pImgInfo);
	if (_pImgFeatures != NULL)
		cvReleaseImage(&_pImgFeatures);
}

void Display::hide()
{
	cvDestroyWindow("Preview");
	cvDestroyWindow("Features");
	cvDestroyWindow("Quality");
	cvWaitKey(100);	// will not close until handled, no guarantee here
	_displayingPreview = false;
	_displayingFeatures = false;
}

void Display::processEvent(int delay)
{
	// this the only way with highgui... (sic)
	cvWaitKey(delay);
}

void Display::showPreview(IplImage *pImage1, IplImage *pImage2)
{
	if (Config::_FeatureDisplay) {
		// create window if necessary
		if (! _displayingPreview) {
			cvNamedWindow("Preview", CV_WINDOW_NORMAL);
			cvResizeWindow("Preview", NBPIXELS_WIDTH, NBPIXELS_HEIGHT*2);
			cvMoveWindow("Preview", FEAT_POSX, FEAT_POSY); // offset from the UL corner of the screen
			_displayingPreview = true;
		}
		// display image
		if (pImage1 != NULL) {
			memcpy(_pImgFeatures->imageData, pImage1->imageData, pImage1->imageSize);
			if (pImage2 != NULL) {
				// assume both have same size !
				memcpy(_pImgFeatures->imageData+(pImage1->imageSize), pImage2->imageData, pImage2->imageSize);
			}
		}
		cvShowImage("Preview", _pImgFeatures);
		cvWaitKey(100);
	}
}

void Display::showFeatures(IplImage *pImage1, IplImage *pImage2, float score)
{
	if (score < _minScore)
		_minScore = score;
	_totalScore += score;
	_totalCount++;

	if (_displayingPreview) {
		cvDestroyWindow("Preview");
		_displayingPreview = false;
	}

	if (Config::_FeatureDisplay) {
		// create windows if not already done
		if (! _displayingFeatures) {
			cvNamedWindow("Features", CV_WINDOW_NORMAL);
			cvResizeWindow("Features", NBPIXELS_WIDTH, NBPIXELS_HEIGHT*2);
			cvMoveWindow("Features", FEAT_POSX, FEAT_POSY); // offset from the UL corner of the screen

			cvNamedWindow("Quality", CV_WINDOW_AUTOSIZE);
			cvMoveWindow("Quality", INFO_POSX, INFO_POSY); // offset from the UL corner of the screen

			_displayingFeatures = true;
		}
	}

	updateScore(score);

	if (_displayingFeatures) {
		// update image buffer by stacking the 2 frames
		memcpy(_pImgFeatures->imageData, pImage1->imageData, pImage1->imageSize);
		memcpy(_pImgFeatures->imageData+(pImage1->imageSize), pImage2->imageData, pImage2->imageSize);
		cvShowImage("Features", _pImgFeatures);
		cvShowImage("Quality", _pImgInfo);
		cvWaitKey(100);

		if (! Config::_FeatureDisplay) {
			// destroy windows - asynchronous, it will be handled in event loop (cvWaitKey)
			cvShowImage("Features", NULL);
			cvShowImage("Quality", NULL);
			cvResizeWindow("Features", 1, 1);
			cvResizeWindow("Quality", 1, 1);
			cvDestroyWindow("Features");
			cvDestroyWindow("Quality");
			cvWaitKey(100);
			_displayingFeatures = false;
		}
	}
	// the opencv windows won't close until handled in event loop, so give a chance each time
	// a longer timeout has no effect here...
	cvWaitKey(1);
}

void Display::showOutOfSync(IplImage *pImage1, IplImage *pImage2)
{
	CvFont font;
	double hScale=2;
	double vScale=2;
	int    lineWidth=2;

	if (_displayingPreview) {
		cvDestroyWindow("Preview");
		_displayingPreview = false;
	}

	if (Config::_FeatureDisplay && _pImgInfo != NULL) {
		// score is not valid
		updateScore(-1);
		cvShowImage("Quality", _pImgInfo);
	}

	if (! _displayingFeatures) {
		// create feature windows
		cvNamedWindow("Features", CV_WINDOW_NORMAL);
		cvResizeWindow("Features", NBPIXELS_WIDTH, NBPIXELS_HEIGHT*2);
		cvMoveWindow("Features", FEAT_POSX, FEAT_POSY); // offset from the UL corner of the screen
		_displayingFeatures = true;
	}
	// update image buffer by stacking the 2 frames
	if (pImage1 != NULL) {
		memcpy(_pImgFeatures->imageData, pImage1->imageData, pImage1->imageSize);
		if (pImage2 != NULL)
			memcpy(_pImgFeatures->imageData+(pImage1->imageSize), pImage2->imageData, pImage2->imageSize);
	}
	// define a font to write some text
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale,vScale, 0, lineWidth);
	cvPutText(_pImgFeatures, "SYNC LOST", cvPoint(40, NBPIXELS_HEIGHT+200), &font, cvScalar(0,0,255));
	cvShowImage("Features", _pImgFeatures);
	cvWaitKey(100);
}

void Display::updateScore(float score)
{
	CvFont font;
	double hScale=2;
	double vScale=2;
	int    lineWidth=2;
	char buf[64];

	// update info display
	if (_pImgInfo != NULL) {
		// reset background
		cvRectangle(_pImgInfo, cvPoint(0,0), cvPoint(INFO_WIDTH,INFO_HEIGHT), cvScalar(0,0,0), CV_FILLED);

		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale,vScale, 0, lineWidth);
		if (score>=0) {
			// display score value
			sprintf(buf, "%d%%", (int)(100*score));
			cvPutText(_pImgInfo, buf, cvPoint(20, 60), &font, cvScalar(255,255,0));
		}
		else {
			// out of sync
			cvPutText(_pImgInfo, "---", cvPoint(20, 60), &font, cvScalar(0,0,255));
		}

		// display score bar
		sprintf(buf, "min:%d%%", (int)(100*_minScore));
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.8, 0.8, 0, 1);
		cvPutText(_pImgInfo, buf, cvPoint(200, 30), &font, getColorScore(_minScore));
		if (_totalCount>0) {
			sprintf(buf, "mean:%d%%", (int)(100*_totalScore/_totalCount));
			cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.8, 0.8, 0, 1);
			cvPutText(_pImgInfo, buf, cvPoint(200, 60), &font, getColorScore(_totalScore/_totalCount));
		}

		cvRectangle(_pImgInfo,
				cvPoint(SCORE_OFFSET_X,SCORE_OFFSET_Y),
				cvPoint(SCORE_OFFSET_X+(score*SCORE_WIDTH), SCORE_OFFSET_Y+SCORE_HEIGHT),
				getColorScore(score),
				CV_FILLED);
	}
}

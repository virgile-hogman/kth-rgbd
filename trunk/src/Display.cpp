#include "cv.h"
#include "highgui.h"

#include "Config.h"
#include "Display.h"
#include "CommonTypes.h"

#define SCORE_OFFSET_X 10
#define SCORE_OFFSET_Y 68
#define SCORE_WIDTH 280
#define SCORE_HEIGHT 10

#define INFO_WIDTH 300
#define INFO_HEIGHT 80

Display::Display()
{
	_displayingFeatures = false;
	// create buffers
	_pImgInfo = cvCreateImage(cvSize(INFO_WIDTH,INFO_HEIGHT),IPL_DEPTH_8U,3);
	_pImgFeatures = cvCreateImage(cvSize(NBPIXELS_WIDTH,NBPIXELS_HEIGHT*2),IPL_DEPTH_8U,3);

}

Display::~Display()
{
	// release buffers
	if (_pImgInfo != NULL)
		cvReleaseImage(&_pImgInfo);
	if (_pImgFeatures != NULL)
		cvReleaseImage(&_pImgFeatures);
}

void Display::showFeatures(FrameData &frameData1, FrameData &frameData2, float score)
{
	CvFont font;
	double hScale=2;
	double vScale=2;
	int    lineWidth=2;

	// update info display
	if (_pImgInfo != NULL) {
		// reset background
		cvRectangle(_pImgInfo, cvPoint(0,0), cvPoint(INFO_WIDTH,INFO_HEIGHT), cvScalar(0,0,0), CV_FILLED);

		// display score value
		char buf[64];
		sprintf(buf, "%d%%", (int)(100*score));
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale,vScale, 0, lineWidth);
		cvPutText(_pImgInfo, buf, cvPoint(20, 60), &font, cvScalar(255,255,0));

		// display score bar
		CvScalar colorScore;
		if (score>0.66)
			colorScore = cvScalar(50,255,50);	// dark green
		else if (score>0.33)
			colorScore = cvScalar (0,165,255);	// orange
		else
			colorScore = cvScalar (0,0,255);	// red

		cvRectangle(_pImgInfo,
				cvPoint(SCORE_OFFSET_X,SCORE_OFFSET_Y),
				cvPoint(SCORE_OFFSET_X+(score*SCORE_WIDTH), SCORE_OFFSET_Y+SCORE_HEIGHT),
				colorScore,
				CV_FILLED);
	}

	if (Config::_FeatureDisplay) {
		// create windows if not already done
		if (! _displayingFeatures) {
			cvNamedWindow("Features", CV_WINDOW_NORMAL);
			cvResizeWindow("Features", NBPIXELS_WIDTH, NBPIXELS_HEIGHT*2);
			cvMoveWindow("Features", 0, 0); // offset from the UL corner of the screen
			cvNamedWindow("Quality", CV_WINDOW_AUTOSIZE);
			cvMoveWindow("Quality", 200, 0); // offset from the UL corner of the screen

			_displayingFeatures = true;
		}
	}

	if (_displayingFeatures) {
		// update display
		IplImage* img1=frameData1.getImage();
		IplImage* img2=frameData2.getImage();
		// update image buffer by stacking the 2 frames
		memcpy(_pImgFeatures->imageData, img1->imageData, img1->imageSize);
		memcpy(_pImgFeatures->imageData+img1->imageSize, img2->imageData, img2->imageSize);
		cvShowImage("Features", _pImgFeatures);
		cvWaitKey(100);

		if (! Config::_FeatureDisplay) {
			// update info as the destroy window is not immediate
			cvShowImage("Quality", _pImgInfo);
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

void Display::showOutOfSync(FrameData &frameData1, FrameData &frameData2)
{
	CvFont font;
	double hScale=2;
	double vScale=2;
	int    lineWidth=2;

	if (Config::_FeatureDisplay && _pImgInfo != NULL) {
		// score is not valid
		cvRectangle(_pImgInfo, cvPoint(0,0), cvPoint(INFO_WIDTH,INFO_HEIGHT), cvScalar(0,0,0), CV_FILLED);
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale,vScale, 0, lineWidth);
		cvPutText(_pImgInfo, "too low", cvPoint(20, 60), &font, cvScalar(0,0,255));
		cvShowImage("Quality", _pImgInfo);
	}

	if (! _displayingFeatures) {
		// create feature windows
		cvNamedWindow("Features", CV_WINDOW_NORMAL);
		cvResizeWindow("Features", NBPIXELS_WIDTH, NBPIXELS_HEIGHT*2);
		cvMoveWindow("Features", 0, 0); // offset from the UL corner of the screen
		_displayingFeatures = true;
	}
	IplImage* img1=frameData1.getImage();
	IplImage* img2=frameData2.getImage();
	// update image buffer by stacking the 2 frames
	memcpy(_pImgFeatures->imageData, img1->imageData, img1->imageSize);
	memcpy(_pImgFeatures->imageData+img1->imageSize, img2->imageData, img2->imageSize);
	// define a font to write some text
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale,vScale, 0, lineWidth);
	cvPutText(_pImgFeatures, "SYNC LOST", cvPoint(40, NBPIXELS_HEIGHT+200), &font, cvScalar(0,0,255));
	cvShowImage("Features", _pImgFeatures);
	cvWaitKey(100);
}

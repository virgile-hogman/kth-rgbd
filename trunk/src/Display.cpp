#include "cv.h"
#include "highgui.h"

#include "Config.h"
#include "Display.h"

Display::Display()
{
	_pImgInfo = cvCreateImage(cvSize(200,80),IPL_DEPTH_8U,3);

	_displayingFeatures = false;
}

Display::~Display()
{
	if (_pImgInfo != NULL)
		cvReleaseImage(&_pImgInfo);
}

void Display::showFeatures(FrameData &frameData1, FrameData &frameData2, float ratio)
{
	CvFont font;
	double hScale=2;
	double vScale=2;
	int    lineWidth=2;

	// update info display
	if (_pImgInfo != NULL) {
		cvRectangle(_pImgInfo, cvPoint(0,0), cvPoint(200,80), cvScalar(0,0,0), CV_FILLED);

		char buf[64];
		sprintf(buf, "%d%%", (int)(100*ratio));
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale,vScale, 0, lineWidth);
		cvPutText(_pImgInfo, buf, cvPoint(20, 60), &font, cvScalar(255,255,0));
	}

	if (Config::_FeatureDisplay) {
		// create windows if not already done
		if (! _displayingFeatures) {
			cvNamedWindow("Frame1", CV_WINDOW_AUTOSIZE);
			cvNamedWindow("Frame2", CV_WINDOW_AUTOSIZE);
			cvNamedWindow("Quality", CV_WINDOW_AUTOSIZE);
			cvMoveWindow("Frame1", 0, 0); // offset from the UL corner of the screen
			cvMoveWindow("Frame2", 0, 400); // offset from the UL corner of the screen
			cvMoveWindow("Quality", 200, 0); // offset from the UL corner of the screen
			_displayingFeatures = true;
		}
	}

	if (_displayingFeatures) {
		// update display
		IplImage* img1=frameData1.getImage();
		IplImage* img2=frameData2.getImage();
		cvShowImage("Frame1", img1);
		cvShowImage("Frame2", img2);
		cvShowImage("Quality", _pImgInfo);
		cvWaitKey(100);

		if (! Config::_FeatureDisplay) {
			// destroy windows
			cvShowImage("Frame1", NULL);
			cvShowImage("Frame2", NULL);
			cvShowImage("Quality", NULL);
			cvResizeWindow("Frame1", 1, 1);
			cvResizeWindow("Frame2", 1, 1);
			cvResizeWindow("Quality", 1, 1);
			cvDestroyWindow("Frame1");
			cvDestroyWindow("Frame2");
			cvDestroyWindow("Quality");
			cvWaitKey(100);
			_displayingFeatures = false;
		}
	}
	cvWaitKey(1);	// the opencv windows won't close until handled here, so give a chance each time
}

void Display::showOutOfSync(FrameData &frameData1, FrameData &frameData2)
{
	CvFont font;
	double hScale=2;
	double vScale=2;
	int    lineWidth=2;

	if (Config::_FeatureDisplay && _pImgInfo != NULL) {
		cvRectangle(_pImgInfo, cvPoint(0,0), cvPoint(200,80), cvScalar(0,0,0), CV_FILLED);
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale,vScale, 0, lineWidth);
		cvPutText(_pImgInfo, "NOK", cvPoint(20, 60), &font, cvScalar(0,0,255));
		cvShowImage("Quality", _pImgInfo);
	}

	if (! _displayingFeatures) {
		// create windows
		cvNamedWindow("Frame1", CV_WINDOW_AUTOSIZE);
		cvNamedWindow("Frame2", CV_WINDOW_AUTOSIZE);
		cvMoveWindow("Frame1", 0, 0); // offset from the UL corner of the screen
		cvMoveWindow("Frame2", 0, 400); // offset from the UL corner of the screen
		_displayingFeatures = true;
	}
	IplImage* img1=frameData1.getImage();
	IplImage* img2=frameData2.getImage();
	// define a font to write some text
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale,vScale, 0, lineWidth);
	cvPutText(img2, "LOST SYNC", cvPoint(40, 200), &font, cvScalar(0,0,255));
	cvShowImage("Frame1", img1);
	cvShowImage("Frame2", img2);
	cvWaitKey(100);
}

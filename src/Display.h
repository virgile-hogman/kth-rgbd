#ifndef DISPLAY_H
#define DISPLAY_H

#include "cv.h"

#include "FrameData.h"

class Display
{
public:
	Display();
	~Display();

	void showFeatures(FrameData &frameData1, FrameData &frameData2, float score);
	void showOutOfSync(FrameData &frameData1, FrameData &frameData2);

private:
	// the feature windows will be displayed if out of sync -or- if config is set for permanent display
	bool	_displayingFeatures;

	IplImage *_pImgInfo;
	IplImage *_pImgFeatures;

};

#endif // DISPLAY_H

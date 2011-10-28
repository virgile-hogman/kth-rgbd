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

#ifndef DISPLAY_H
#define DISPLAY_H

#include "cv.h"

#include "FrameData.h"

class Display
{
public:
	Display();
	~Display();

	void close();
	void processEvent(int delay);
	void showFeatures(FrameData &frameData1, FrameData &frameData2, float score);
	void showOutOfSync(FrameData &frameData1, FrameData &frameData2);

protected:
	void updateScore(float score);

	// the feature windows will be displayed if out of sync -or- if config is set for permanent display
	bool	_displayingFeatures;

	IplImage *_pImgInfo;
	IplImage *_pImgFeatures;

	float _totalScore;
	float _totalCount;
	float _minScore;
};

#endif // DISPLAY_H

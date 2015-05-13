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

class Display
{
public:
	Display();
	~Display();

	// returns keycode
	int hide();
	// returns keycode
	int processEvent();
	// returns keycode
	int showPreview(IplImage *pImage1, IplImage *pImage2);
	// returns keycode
	int showFeatures(IplImage *pImage1, IplImage *pImage2, float score);
	// returns keycode
	int showOutOfSync(IplImage *pImage1, IplImage *pImage2);

protected:
	void updateScore(float score);

	// the feature windows will be displayed if out of sync -or- if config is set for permanent display
	bool	_displayingFeatures;
	bool	_displayingPreview;

	IplImage *_pImgInfo;
	IplImage *_pImgFeatures;

	float _totalScore;
	float _totalCount;
	float _minScore;
};

#endif // DISPLAY_H

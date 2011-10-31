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

#ifndef MAP_H
#define MAP_H

#include "FrameData.h"
#include "Graph.h"
#include "Display.h"

#include <vector>
#include <fstream>

class Map
{
public:
	// start of a new transformation sequence
	void startSequence();

	// end of sequence updates display
	void stopSequence();

	// add a pair of frames to the transformation sequence
	bool addFrames(int frameID1, int frameID2, Transformation &transform);

	// transform a full sequence
	void addSequence(std::vector<int> &sequenceFramesID);

	// restore transformation from archive with given range
	void restoreSequence(int minFrameID, int maxFrameID);

	// build the map from the current transformation sequence
	void build();

	// regenerate PCD files only from existing graph
	void regeneratePCD();

	Display *getDisplay()			{ return &_display; }

private:
	bool detectLoopClosure(const PoseVector	&cameraPoses);

private:
	// graph optimizer
	Graph					_graphOptimizer;

	// sequence of transformations
	TransformationVector	_sequenceTransform;

	// archive file to save the transformations
	std::ofstream 			_fileTransformOut;

	// buffers
	FrameData				_bufferFrameData1;
	FrameData				_bufferFrameData2;

	// display windows
	Display					_display;
};

#endif // MAP_H


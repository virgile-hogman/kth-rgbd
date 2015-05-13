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

#ifndef SEQUENCE_H
#define SEQUENCE_H

#include "FrameData.h"
#include "Graph.h"
#include "Display.h"

#include <vector>
#include <fstream>

class Sequence
{
public:
	// record a new sequence with camera, compute transfo and build map
	void recordFrames(bool buildMap);

	// reload a full sequence of frames in a given range, transformations will be recomputed
	bool reloadFrames(int min, int max);

	// restore transformation from archive with given range
	void restoreTransformations(int minFrameID, int maxFrameID);

	// restore an initial graph
	void restoreInitialGraph();

	// build the map from the current transformation sequence
	void buildMap();

	// optimize map with loop closures already defined in the initial graph
	void optimizeMap();

		// regenerate PCD files only from existing graph (initial and optimized if available)
	void regeneratePCD();

private:
	// start of a new transformation sequence
	void startTransform();

	// end of sequence updates display
	void stopTransform();

	// add a pair of frames to the transformation sequence, the result transform is returned
	bool addFramesTransform(int frameID1, int frameID2, Transformation &transform, int &keywait);

	// detect the loop closures and updates the graph
	bool detectLoopClosure(const PoseVector	&cameraPoses);

private:
	// graph optimizer
	Graph					_graphOptimizer;

	// sequence of transformations
	TransformationVector	_sequenceTransform;

	// archive file to save the transformations
	std::ofstream 			_fileTransformOut;

	// buffers
	FrameData				_bufferFrame1;
	FrameData				_bufferFrame2;

	// display windows
	Display					_display;
};

#endif // SEQUENCE_H


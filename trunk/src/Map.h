#ifndef MAP_H
#define MAP_H

#include "FrameData.h"
#include "Graph.h"


#include <vector>
#include <fstream>

class Map
{
public:
	// start a transformation sequence
	void initSequence();

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

private:
	void detectLoopClosure(const PoseVector	&cameraPoses);

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
};

#endif // MAP_H


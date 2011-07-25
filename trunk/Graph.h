#ifndef __GRAPH_H__
#define __GRAPH_H__

#include "CommonTypes.h"
#include "g2o/core/graph_optimizer_sparse.h"
#include <string>

class Graph
{
public:
	Graph();
	
	void initialize();
	
	void addVertex(const Pose &pose);
	void addEdge(const Transformation &transfo);
	
	void optimize();
	
	bool extractPose(Pose &pose);
	bool extractAllPoses(PoseVector &poses);

	void load(const char *directory);
	void save(const char *directory);
	
private:
	g2o::SparseOptimizer	_optimizer;
};

#endif // __GRAPH_H__

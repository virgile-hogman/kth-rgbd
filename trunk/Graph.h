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
	
	void addVertex(int id, const Eigen::Matrix4f &pose);
	void addEdge(int id1, int id2, const Transformation &transfo);
	
	void optimize();
	
	bool getPose(int id, Pose &pose);
	
	void save(const char *directory);
	
private:
	g2o::SparseOptimizer	_optimizer;
};

#endif // __GRAPH_H__

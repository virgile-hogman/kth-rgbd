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
	
	void addVertex(int id, const Eigen::Matrix4f &transfo);
	void addEdge(int id1, int id2, const Transformation &transfo);
	
	void optimize();
	
	void getTransfo(int id, Transformation& transfo);
	
	void save(const char *directory);
	void setSaveDirectory(const std::string &directory) { _saveDirectory = directory; }
	
private:
	g2o::SparseOptimizer	_optimizer;
	
	std::string				_saveDirectory;

};

#endif // __GRAPH_H__
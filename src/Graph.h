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

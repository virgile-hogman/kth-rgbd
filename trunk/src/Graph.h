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
#include "g2o/core/sparse_optimizer.h"
#include <string>

class Graph
{
public:
	Graph();
	
	// initialize solver
	void initialize();
	
	void addVertex(const Pose &pose);
	void addEdge(const Transformation &transfo);
	
	// returns the effective nb of iterations or -1 if failed to start
	int optimize();
	
	bool extractPose(Pose &pose);
	bool extractAllPoses(PoseVector &poses);

	void load(const char *directory);
	void save(const char *directory);
	
private:
	bool					_initialized;
	g2o::SparseOptimizer	_optimizer;
};

#endif // __GRAPH_H__

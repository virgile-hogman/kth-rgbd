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

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/se3quat.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "Config.h"
#include "Graph.h"

#include <list>

#define NB_ITERATIONS 15	//  for g2o optimizer

Graph::Graph()
{
	_initialized =false;
}

void Graph::initialize()
{
	if (! _initialized) {	
		_optimizer.setVerbose(true);

		typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
		typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

		SlamLinearSolver* linearSolver = new SlamLinearSolver();
		linearSolver->setBlockOrdering(false);
		SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);

		g2o::OptimizationAlgorithmLevenberg* solverLevenberg = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
		_optimizer.setAlgorithm(solverLevenberg);

		_initialized = true;
	}
}

void Graph::addVertex(const Pose &pose)
{
	Eigen::Affine3f eigenTransform(pose._matrix);
	Eigen::Quaternionf eigenRotation(eigenTransform.rotation());

	g2o::SE3Quat poseSE3(
			// NOTE the order of the arguments : w comes first!
			Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),
			Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
	
	// add new vertex to the graph
	g2o::VertexSE3 *vertexSE3 = new g2o::VertexSE3();
	vertexSE3->setId(pose._id);
	vertexSE3->setEstimate(poseSE3);
	
	if (_optimizer.vertices().size()==0)
		vertexSE3->setFixed(true);	// fix the first vertex only

	_optimizer.addVertex(vertexSE3);
}

void Graph::addEdge(const Transformation &transfo)
{
	Eigen::Affine3f eigenTransform(transfo._matrix);
	Eigen::Quaternionf eigenRotation(eigenTransform.rotation());

	g2o::SE3Quat transfoSE3(
			// NOTE the order of the arguments : w comes first!
			Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),
			Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));

	// add new edge to the graph
	g2o::EdgeSE3* edgeSE3 = new g2o::EdgeSE3;
	edgeSE3->vertices()[0] = _optimizer.vertex(transfo._idOrig);	// observer
	edgeSE3->vertices()[1] = _optimizer.vertex(transfo._idDest);	// observed
	edgeSE3->setMeasurement(transfoSE3.inverse());		// how to move from observer to observed => inverse transfo

	Eigen::Matrix<double, 6, 6, 0, 6, 6> mat;
	mat.setIdentity(6,6);
	edgeSE3->information() = mat;

	_optimizer.addEdge(edgeSE3);
}

int Graph::optimize()
{
	// initial guess
	if (! _optimizer.initializeOptimization())
		return -1;

	return _optimizer.optimize(NB_ITERATIONS); 	// returns the effective nb of iterations
}

bool Graph::extractPose(Pose &pose)
{
	g2o::VertexSE3 *vertexSE3 = NULL;
	
	vertexSE3 = (g2o::VertexSE3*)_optimizer.vertex(pose._id);
	if (vertexSE3 != NULL)
	{
		// get internal matrix representation through Eigen::Isometry3d
		Eigen::Matrix4d mat = vertexSE3->estimate().matrix(); 
		// downcast to float
		pose._matrix = mat.cast<float>();
		return true;
	}

	pose._matrix = Eigen::Matrix4f::Identity();
	return false;
}

bool Graph::extractAllPoses(PoseVector &poses)
{
	Pose pose;
	std::list<int> listID;

	// using the iterator, the vertices are not returned in the same order (compare function not on id?)
	// a temporary list is used to sort them according to the id
	poses.clear();
    for (g2o::HyperGraph::VertexIDMap::iterator it=_optimizer.vertices().begin();
    		it!=_optimizer.vertices().end();
    		it++)
    {
    	listID.push_back(it->first);
    }
    listID.sort();

    for (std::list<int>::iterator it=listID.begin(); it!=listID.end(); ++it)
    {
    	// id must be set before
    	pose._id = *it;
		if (! extractPose(pose))
		{
			std::cerr << "Error in graph extraction!" << std::endl;
			return false;
		}
    	poses.push_back(pose);
    }
    return true;
}

void Graph::load(const char* filename)
{
	// save the graph
	char buf_graph[256];
	sprintf(buf_graph, "%s/%s", Config::_PathDataProd.c_str(), filename);
	std::ifstream file(buf_graph);

	_optimizer.clear();
	_optimizer.load(file);
}

void Graph::save(const char* filename)
{
	// save the graph
	char buf_graph[256];
	sprintf(buf_graph, "%s/%s", Config::_PathDataProd.c_str(), filename);
	std::ofstream file(buf_graph);

	_optimizer.save(file);
}

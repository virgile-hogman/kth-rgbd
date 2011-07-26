#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/math_groups/se3quat.h"
#include "g2o/core/structure_only_solver.h"
#include "g2o/types/slam3d/vertex_se3_quat.h"
#include "g2o/types/slam3d/edge_se3_quat.h"

#include "Config.h"
#include "Graph.h"

#include <list>

Graph::Graph()
{
}

void Graph::initialize()
{
	_optimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
	_optimizer.setVerbose(true);

	g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
	linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
	g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(&_optimizer,linearSolver);
	_optimizer.setSolver(solver_ptr);
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
	vertexSE3->estimate() = poseSE3;
	
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
	edgeSE3->setInverseMeasurement(transfoSE3);

	Eigen::Matrix<double, 6, 6, 0, 6, 6> mat;
	mat.setIdentity(6,6);
	edgeSE3->information() = mat;

	_optimizer.addEdge(edgeSE3);
}

void Graph::optimize()
{
	// initial guess
	_optimizer.initializeOptimization();
	// iterations
	_optimizer.setVerbose(true);
	_optimizer.optimize(5);
}

bool Graph::extractPose(Pose &pose)
{
	g2o::VertexSE3 *vertexSE3 = NULL;
	
	vertexSE3 = (g2o::VertexSE3*)_optimizer.vertex(pose._id);
	if (vertexSE3 != NULL)
	{
		Eigen::Matrix4d mat = vertexSE3->estimate().to_homogenious_matrix();
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
	sprintf(buf_graph, "%s/%s", Config::_ResultDirectory.c_str(), filename);
	std::ifstream file(buf_graph);

	_optimizer.clear();
	_optimizer.load(file);
}

void Graph::save(const char* filename)
{
	// save the graph
	char buf_graph[256];
	sprintf(buf_graph, "%s/%s", Config::_ResultDirectory.c_str(), filename);
	std::ofstream file(buf_graph);

	_optimizer.save(file);
}

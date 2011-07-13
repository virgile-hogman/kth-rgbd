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
	g2o::VertexSE3 *vertexSE3 = NULL;

	Eigen::Affine3f eigenTransform(pose._matrix);
	Eigen::Quaternionf eigenRotation(eigenTransform.rotation());

	g2o::SE3Quat poseSE3(
    		// NOTE the order of the arguments : w comes first!
			Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),
			Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));
	
	// add new vertex to the graph
	vertexSE3 = new g2o::VertexSE3();
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

	int id1 = transfo._idOrig;
	int id2 = transfo._idDest;

	std::cout << "addEdge id1=" << id1 << " id2=" << id2 << "\n";
	fflush(stdout);

	g2o::SE3Quat transfoSE3(
			// NOTE the order of the arguments : w comes first!
			Eigen::Quaterniond(eigenRotation.w(), eigenRotation.x(), eigenRotation.y(), eigenRotation.z()),
			Eigen::Vector3d(eigenTransform(0,3), eigenTransform(1,3), eigenTransform(2,3)));

	// add new edge to the graph
	g2o::EdgeSE3* edgeSE3 = new g2o::EdgeSE3;
	edgeSE3->vertices()[0] = _optimizer.vertex(id1);	// observer
	edgeSE3->vertices()[1] = _optimizer.vertex(id2);	// observed
	edgeSE3->setMeasurement(transfoSE3.inverse());		// how to move from observer to observed => inverse transfo
	edgeSE3->setInverseMeasurement(transfoSE3);

	Eigen::Matrix<double, 6, 6, 0, 6, 6> mat;
	mat.setIdentity(6,6);
	edgeSE3->information() = mat;

	_optimizer.addEdge(edgeSE3);
}

void Graph::optimize()
{
	_optimizer.initializeOptimization();
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
	// extract the updated camera position from the optimized graph
	for (int i=0; i<poses.size(); i++)
	{
		// get the pose from the graph
		if (! extractPose(poses[i]))
		{
			std::cerr << "Error in graph extraction!" << std::endl;
			// cut the remaining poses, they are not valid anymore
			poses.resize(i);
			return false;
		}
	}
	return true;
}

void Graph::save(const char* filename)
{
	// save the graph
	char buf_graph[256];
	sprintf(buf_graph, "%s/%s", Config::_ResultDirectory.c_str(), filename);
	std::ofstream foutopt(buf_graph);
	_optimizer.save(foutopt);
}

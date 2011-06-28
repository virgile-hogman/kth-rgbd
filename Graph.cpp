

//#include "g2o/types/slam3d/types_six_dof_quat.h"
#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
//#include "g2o/solvers/dense/linear_solver_dense.h"
//#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/math_groups/se3quat.h"
#include "g2o/core/structure_only_solver.h"

#include "g2o/types/slam3d/vertex_se3_quat.h"
#include "g2o/types/slam3d/edge_se3_quat.h"

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


void Graph::addVertex(int id, const Eigen::Matrix4f &transfo)
{
	g2o::VertexSE3 *vertexSE3 = NULL;
	
    Eigen::Affine3f eigen_transformCumulated(transfo);
	
    Eigen::Affine3f eigen_transformCumulatedInv = eigen_transformCumulated;
    Eigen::Quaternionf eigen_quat_cumulated(eigen_transformCumulatedInv.rotation());
    
    std::cerr << "Camera mat inv\n" << eigen_transformCumulatedInv.matrix() << std::endl;
      
    g2o::SE3Quat pose(
    		// NOTE the order of the arguments : w comes first!
			Eigen::Quaterniond(eigen_quat_cumulated.w(), eigen_quat_cumulated.x(), eigen_quat_cumulated.y(), eigen_quat_cumulated.z()),
			Eigen::Vector3d(eigen_transformCumulatedInv(0,3), eigen_transformCumulatedInv(1,3), eigen_transformCumulatedInv(2,3)));
	
	// add new vertex to the graph
    vertexSE3 = new g2o::VertexSE3();
	vertexSE3->setId(id);
	vertexSE3->estimate() = pose;
	_optimizer.addVertex(vertexSE3);

	/*Eigen::Matrix4f mat = vertexSE3->estimate().to_homogenious_matrix().cast<float>();
	//Eigen::Matrix4f mat = pose.to_homogenious_matrix().cast<float>();
	// downcast to float
    std::cerr << "Camera mat SE3\n" << mat << std::endl;*/
}

void Graph::addEdge(int id1, int id2, const Transformation &transfo)
{
    //SE2 transf = transfo._matrix.inverse();
    Eigen::Affine3f eigen_transform(transfo._matrix);			
    Eigen::Quaternionf eigen_quat(eigen_transform.rotation());
    Eigen::Vector3d eigen_translation(transfo._matrix(0, 3), transfo._matrix(1, 3), transfo._matrix(2, 3));
    Eigen::Quaterniond eigen_rotation(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
    
    g2o::SE3Quat transf(eigen_rotation, eigen_translation);
    
	// add new edge to the graph
    g2o::EdgeSE3* edgeSE3 = new g2o::EdgeSE3;
    edgeSE3->vertices()[0] = _optimizer.vertex(id1);
    edgeSE3->vertices()[1] = _optimizer.vertex(id2);
    edgeSE3->setMeasurement(transf);
    edgeSE3->setInverseMeasurement(transf.inverse());
    //edgeSE3->setInformation(information);
    _optimizer.addEdge(edgeSE3);
}

void Graph::getTransfo(int id, Transformation& transfo)
{
	g2o::VertexSE3 *vertexSE3;
	
    std::cerr << "Get transfo from frameId = " << id << std::endl;
     
	vertexSE3 = (g2o::VertexSE3*)_optimizer.vertex(id);
	
	if (vertexSE3 != NULL)
	{
		transfo._isValid = true;
		transfo._idOrig = -1;
		transfo._idDest = id;
	
		Eigen::Matrix4d mat = vertexSE3->estimate().to_homogenious_matrix();
		// downcast to float
		transfo._matrix = mat.cast<float>();
	}
}

void Graph::optimize()
{
    _optimizer.initializeOptimization();
    _optimizer.setVerbose(true);
    _optimizer.optimize(10);
}

void Graph::save(const char* filename)
{
	// save the graph
	char buf_graph[256];
	sprintf(buf_graph, "%s/%s", _saveDirectory.c_str(), filename);
	std::ofstream foutopt(buf_graph);
    _optimizer.save(foutopt);	
}
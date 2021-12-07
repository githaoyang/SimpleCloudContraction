#pragma once
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>


#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>  //滤波相关
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <Eigen/src/Eigenvalues/EigenSolver.h>

#include <pcl/common/io.h>

#include <cmath>
#include <unordered_map>
#include <vector>
#include <queue>
#include <stack>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;
using namespace pcl;

class skeletonPoint;
class Graph;
struct VertexNode;
struct ArcNode;
typedef ArcNode* ArcNodePtr;

class extractSkeleton
{
public:
	extractSkeleton();
	//用于提取关键点时准备所需数据结构
	void prepareExtractNodePoint(PointCloudT::Ptr cloud);
	int extractCurrentNodePoint();
	void findNextNode(vector<int>& pointIdxRadiusSearch, vector<float>& pointRadiusSquaredDistance);
	void connectSkeltonNode(int circleSize);
	void removeDuplicatedNode();
	void visitAllNodes();

	//用于连接节点时准备所需数据（图）结构
	void prepareNodesDataStructure();

	pcl::KdTreeFLANN<PointT> kdtree;
	PointCloudT::Ptr cloud;							//点云指针 操作点云
	PointCloudT::Ptr localNodeCloud;
	PointT min_point_AABB;
	PointT max_point_AABB;
	PointT searchPoint;
	float circleSize;
	int cloudIteration;
	int nodeIteration;
	int nodeDensity; 
	int maxHeightPointIndice;

	//用于提取关键点时判断节点是否访问过
	vector<bool> isPointVisited;
	//用于判断node节点的队列
	queue<PointT> nextNode;
	//node列表
	vector<PointT> nodeList;
	//node节点图结构指针
	Graph* skeletonGraph;

private:
	int averageLength;
};


//提取骨骼点所用的图结构
class Graph
{
public:
	vector<VertexNode*> adjList;
	int vexNum;
	int edgeNum;	
};


// 定义边表结点
struct ArcNode
{
	int adjvex;// 邻接点域
	ArcNodePtr next;
};


// 定义顶点表结点
struct VertexNode
{
	VertexNode(int _vertex) :vertex(_vertex) {};
	int vertex;
	ArcNodePtr firstedge = nullptr;
	bool isVisited = false;
	bool isConnected = false;
};

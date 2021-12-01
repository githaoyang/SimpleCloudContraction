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

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;
using namespace pcl;

class skeletonPoint;
class Graph;

class extractSkeleton
{
public:
	extractSkeleton();

	void prepareSkeletonDataStructure(PointCloudT::Ptr cloud);
	int extractCurrentSkePoint();
	void findNextNode(vector<int>& pointIdxRadiusSearch, vector<float>& pointRadiusSquaredDistance);
	void connectSkeltonNode();
	void prepareNodesDataStructure();

	pcl::KdTreeFLANN<PointT> kdtree;
	PointCloudT::Ptr cloud;							//点云指针 操作点云
	PointT min_point_AABB;
	PointT max_point_AABB;
	PointT searchPoint;
	float circleSize;
	int cloudIteration;
	int nodeIteration;
	int nodeDensity;

	vector<bool> isPointVisited;
	queue<PointT> nextNode;
	vector<PointT> nodeList;

private:
	PointCloudT::Ptr localNodeCloud;

};


class skeletonPoint
{
public:

	skeletonPoint(int i, bool flag) :indice(i), isVisited(flag) {};

	int indice;
	bool isVisited = false;

};

class Graph
{
public:
	int vertex;
	vector<Graph*> next;
	Graph(int _vertex)
	{
		vertex = _vertex;
		next.emplace_back(NULL);
	}
	bool isVisited;
};
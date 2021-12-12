#pragma once
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>


#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>  //�˲����
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
#include <set>
#include "dataManage.h"

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
	//������ȡ�ؼ���ʱ׼���������ݽṹ
	void prepareExtractNodePoint(PointCloudT::Ptr cloud, dataManager* cloudManager);
	int extractCurrentNodePoint();
	void extractOneListNode();
	void findNextNode(vector<int>& pointIdxRadiusSearch, vector<float>& pointRadiusSquaredDistance);
	void connectSkeltonNode(int circleSize);
	void removeDuplicatedNode();
	void visitAllNodes();
	void getLeftPointCloud();
	void getContractedNodeNeighborPoints(int indice);
	void resetParameter();
	dataManager* cloudManager;

	//�������ӽڵ�ʱ׼���������ݣ�ͼ���ṹ
	void prepareNodesDataStructure();

	pcl::KdTreeFLANN<PointT> kdtree;
	PointCloudT::Ptr cloud;							//����ָ�� ��������
	PointCloudT::Ptr localNodeCloud;
	PointCloudT::Ptr bodyCloud;
	PointT min_point_AABB;
	PointT max_point_AABB;
	PointT searchPoint;
	float circleSize;
	int cloudIteration;
	int nodeIteration;
	int nodeDensity; 
	int maxHeightPointIndice;
	set<int> contractionFinishedPartNodeNeighborPoints;

	//������ȡ�ؼ���ʱ�жϽڵ��Ƿ���ʹ�
	vector<bool> isPointVisited;
	//�����ж�node�ڵ�Ķ���
	queue<PointT> nextNode;
	//node�б�
	vector<PointT> nodeList; 
	vector<PointT> selectedNodeList;
	vector<float> nodeListLinearity;
	vector<float> nodeListNeighborAverageLinearity;
	//node�ڵ�ͼ�ṹָ��
	Graph* skeletonGraph;

private:
	int averageLength;
};


//��ȡ���������õ�ͼ�ṹ
class Graph
{
public:
	vector<VertexNode*> adjList;
	int vexNum;
	int edgeNum;	
};


// ����߱���
struct ArcNode
{
	int adjvex;// �ڽӵ���
	ArcNodePtr next;
};


// ���嶥�����
struct VertexNode
{
	VertexNode(int _vertex) :vertex(_vertex) {};
	int vertex;
	ArcNodePtr firstedge = nullptr;
	bool isVisited = false;
	int inNode = 0;
	bool isConnected = false;
};

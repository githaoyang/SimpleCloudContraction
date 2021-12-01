#pragma once
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>  //滤波相关
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>   //基于采样一致性分割的类的头文件
#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include "QVTKWidget.h"
#include <vtkRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/common/io.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <Eigen/src/Eigenvalues/EigenSolver.h>

#include <numeric>
#include <thread>
#include <cmath>
#include <unordered_set>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

class dataManager
{
public:
	dataManager();

	int cloudIteration;								//用于遍历每个点，点云中点的序列号
	float KNNcontraction_K;							//用于KNN收缩的邻居数
	float PCARadiusSize;							//用于PCA找主特征向量的邻域大小，为半径值
	int downSamplingNeigborSize;					//体素下采样体素大小
	float linearity;								//线性度，用于判断是否此点停止收缩
	int voxelSize;
	unordered_set<int> finishedContractionPoints;	//完成收缩的点云列表
	PointCloudT::Ptr cloud;							//点云指针 操作点云
	PointCloudT::Ptr rawCloud;						//点云指针 原始读入点云
	PointCloudT::Ptr nextFrameCloud;
	PointCloudT::Ptr lastFrameCloud;
	PointCloudT::Ptr voxelCloud;
	Eigen::Vector3f local_displacement_vector;		//KNN范围内的当前点相对于其他点的位移向量
	Eigen::Vector3f field_move_vector;				//实际位移向量
	PointT min_point_AABB;
	PointT max_point_AABB;
	vector<Eigen::Vector3f> voxelCenter;

	void onePointContraction(vector<int>& pointIdxKNNSearch);
	void onePointContraction(vector<int>& pointIdxKNNSearch, vector<int>& pointIdxRadiusSearch);
	void pointsContraction();
	void pointsContraction(int pointNeighborSize);
	int openPointCloud(string path);
	void getAverageNeigborSize(int pointSerialNumber, int pointNeighborSize =15);
	void removeCloseAroundNeighborPoints();
	void downSamplingNeighborPoints(int filterSize = 10);
	void startVoxelTransform();

private:
	PointT searchPoint;
};
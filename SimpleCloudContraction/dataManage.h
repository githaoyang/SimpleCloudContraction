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
#include <pcl/filters/radius_outlier_removal.h>  //�˲����
#include <pcl/registration/icp.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>   //���ڲ���һ���Էָ�����ͷ�ļ�
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

	int cloudIteration;								//���ڱ���ÿ���㣬�����е�����к�
	float KNNcontraction_K;							//����KNN�������ھ���
	float PCARadiusSize;							//����PCA�������������������С��Ϊ�뾶ֵ
	int downSamplingNeigborSize;					//�����²������ش�С
	float linearity;								//���Զȣ������ж��Ƿ�˵�ֹͣ����
	int voxelSize;
	unordered_set<int> finishedContractionPoints;	//��������ĵ����б�
	PointCloudT::Ptr cloud;							//����ָ�� ��������
	PointCloudT::Ptr rawCloud;						//����ָ�� ԭʼ�������
	PointCloudT::Ptr nextFrameCloud;
	PointCloudT::Ptr lastFrameCloud;
	PointCloudT::Ptr voxelCloud;
	Eigen::Vector3f local_displacement_vector;		//KNN��Χ�ڵĵ�ǰ��������������λ������
	Eigen::Vector3f field_move_vector;				//ʵ��λ������
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
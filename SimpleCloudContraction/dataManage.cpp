#include "dataManage.h"

dataManager::dataManager()
{
	cloud.reset(new PointCloudT);
	rawCloud.reset(new PointCloudT);
	lastFrameCloud.reset(new PointCloudT);
	voxelCloud.reset(new PointCloudT);
	lastFrameCloud->resize(1);

	pcl::io::loadPCDFile("G:/person4.pcd", *rawCloud);
	pcl::copyPointCloud(*rawCloud, *cloud);


	cloudIteration = 0;
	KNNcontraction_K = 10;
	PCARadiusSize = 50;
	downSamplingNeigborSize = 120;
	linearity = 0.8;
	voxelSize = 10;
	principal_Directivity.assign(rawCloud->size(), 0);
	getAverageNeigborSize(cloud->size() / 2);


}


int dataManager::openPointCloud(string path)
{
	return pcl::io::loadPCDFile(path, *rawCloud);
}

void dataManager::onePointContraction(vector<int>& neighborPoints)
{
	//��ȡPCA�ֲ��������
	pcl::PointCloud<PointT>::Ptr localCloud(new pcl::PointCloud<PointT>);
	for (auto i : neighborPoints)
	{
		localCloud->points.emplace_back((*cloud)[i]);
	}
	//������������
	pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
	feature_extractor.setInputCloud(localCloud);
	feature_extractor.compute();

	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center; 
	
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	principal_Directivity[cloudIteration] = major_value / (major_value + middle_value + minor_value);
	if (principal_Directivity[cloudIteration] > linearity)
	{
		finishedContractionPoints.emplace(cloudIteration);
		(*cloud)[cloudIteration].r = 0;
		(*cloud)[cloudIteration].g = 255;
		(*cloud)[cloudIteration].b = 0;
		return;
	}
	//����KNN
	local_displacement_vector[0] =0;
	local_displacement_vector[1] =0;
	local_displacement_vector[2] = 0;
	field_move_vector[0] = 0;
	field_move_vector[1] = 0;
	field_move_vector[2] = 0;
	int neighborPointSize = neighborPoints.size();
	for (int i = 0; i < neighborPointSize; i++)
	{
		local_displacement_vector[0] += ((*cloud)[neighborPoints[i]].x - searchPoint.x);
		local_displacement_vector[1] += ((*cloud)[neighborPoints[i]].y - searchPoint.y);
		local_displacement_vector[2] += ((*cloud)[neighborPoints[i]].z - searchPoint.z);
	}
	//u
	local_displacement_vector[0] = local_displacement_vector[0] / KNNcontraction_K;
	local_displacement_vector[1] = local_displacement_vector[1] / KNNcontraction_K;
	local_displacement_vector[2] = local_displacement_vector[2] / KNNcontraction_K;

	float result_Multi_Vector = local_displacement_vector[0]*major_vector[0]
		+ local_displacement_vector[1]*major_vector[1] + local_displacement_vector[2]*major_vector[2];
	float vectorNorm_local_displacement_vector = sqrt(local_displacement_vector[0]*local_displacement_vector[0]
		+ local_displacement_vector[1]*local_displacement_vector[1] + local_displacement_vector[2]*local_displacement_vector[2]);
	float vectorNorm_major_vector = sqrt(major_vector[0]*major_vector[0]
		+ major_vector[1]*major_vector[1] + major_vector[2]*major_vector[2]);
	float cosANGLE = abs(result_Multi_Vector /(vectorNorm_local_displacement_vector*vectorNorm_major_vector));

	field_move_vector = vectorNorm_local_displacement_vector * cosANGLE*(1 - principal_Directivity[cloudIteration])*major_vector
		+ (local_displacement_vector - vectorNorm_local_displacement_vector * cosANGLE * major_vector);

	//�����ƶ�����µ�
	searchPoint.x = searchPoint.x + field_move_vector[0];
	searchPoint.y = searchPoint.y + field_move_vector[1];
	searchPoint.z = searchPoint.z + field_move_vector[2];
	//searchPoint.a = 0.9;
	//searchPoint.r = 255;
	//newLocalPoint.g = 255;
	//newLocalPoint.b = 255;

	(*cloud)[cloudIteration] = searchPoint;

}


//һ�ε�������
void dataManager::pointsContraction()
{
	//����Ѿ��ﵽ����Ҫ���ڱ��κ�֮��ĵ�����ֹͣ����
	if (finishedContractionPoints.count(cloudIteration) == 0)
	{
		pcl::KdTreeFLANN<PointT> kdtree;
		kdtree.setInputCloud(cloud);
		getAverageNeigborSize(cloudIteration, KNNcontraction_K);
		int radius = PCARadiusSize * 3;
		int K = KNNcontraction_K;
		//K���ڽ����ƽ�������������Χ�ڵĵ� ���ڼ���PCA������
		searchPoint = (*cloud)[cloudIteration];

		std::vector<int> pointIdxKNNSearch(K);
		std::vector<float> pointKNNSquaredDistance(K);
		kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);
		onePointContraction(pointIdxKNNSearch);
	}

	if (cloudIteration < cloud->size() - 1)
	{
		cloudIteration++;
	}
	else
	{
		cloudIteration = 0;
	}
}


void dataManager::getAverageNeigborSize(int pointSerialNumber, int pointNeighborSize)
{
	if (cloud->points.size() < 30)
	{
		return;
	}
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);
	PointT searchPoint = (*cloud)[pointSerialNumber];

	std::vector<int> pointIdxKNNSearch(pointNeighborSize);
	std::vector<float> pointKNNSquaredDistance(pointNeighborSize);
	kdtree.nearestKSearch(searchPoint, pointNeighborSize, pointIdxKNNSearch, pointKNNSquaredDistance);
	PCARadiusSize = sqrt(accumulate(pointKNNSquaredDistance.begin(), pointKNNSquaredDistance.end(), 0) / pointNeighborSize);
}

void dataManager::removeCloseAroundNeighborPoints()
{
	int cloudSize = cloud->points.size();
	for(int i = 0; i < cloudSize; i++)
	{
		if ((*cloud)[i].a > 0)
		{
			PointT newPoint;
			newPoint.x = (*cloud)[i].x;
			newPoint.y = (*cloud)[i].y;
			newPoint.z = (*cloud)[i].z;
			newPoint.r = 255;
			newPoint.g = 255;
			newPoint.b = 255;
			nextFrameCloud->push_back(newPoint);
		}
	}
	copyPointCloud(*nextFrameCloud, *cloud);
	nextFrameCloud.reset(new PointCloudT);
	nextFrameCloud->resize(1);
}

void dataManager::downSamplingNeighborPoints(int filterSize)
{
 	pcl::VoxelGrid<PointT> sor;    // �����˲����²���������
	sor.setInputCloud(cloud);             //������Ҫ���˵ĵ���
	//�����˲�ʱ�������������Ϊxcm��������,
	sor.setLeafSize(filterSize, filterSize, filterSize);
	sor.filter(*cloud);          //ִ���˲����������������
}


void dataManager::startVoxelTransform()
{
	pcl::copyPointCloud(*cloud, *voxelCloud);
	
	downSamplingNeighborPoints(voxelSize);
	
	//��ȡ�����С���Σ����ں�ʱ
/*	pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);*/
	voxelCenter.clear();
	for (int i = 0; i < cloud->points.size(); i++)
	{
		double x = cloud->points[i].x;
		double y = cloud->points[i].y;
		double z = cloud->points[i].z;

		Eigen::Vector3f center(floor(x / voxelSize)*voxelSize + voxelSize / 2, 
			floor(y / voxelSize)*voxelSize + voxelSize / 2, floor(z / voxelSize)*voxelSize + voxelSize / 2);
		voxelCenter.emplace_back(center);
	}
}

void dataManager::resetParameters()
{
	finishedContractionPoints.clear();
	principal_Directivity.clear();
	voxelCenter.clear();
}

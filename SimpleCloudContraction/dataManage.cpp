#include "dataManage.h"

dataManager::dataManager()
{
	cloud.reset(new PointCloudT);
	rawCloud.reset(new PointCloudT);
	nextFrameCloud.reset(new PointCloudT);
	voxelCloud.reset(new PointCloudT);
	nextFrameCloud->resize(1);
	cloud->resize(1);
	rawCloud->resize(1);
	pcl::io::loadPCDFile("G:/bodyleftForcontraction.pcd", *rawCloud);
	pcl::copyPointCloud(*rawCloud, *cloud);


	cloudIteration = 0;
	KNNcontraction_K = 5;
	PCARadiusSize = 50;
	downSamplingNeigborSize = 50;
	linearity = 0.8;
	voxelSize = 10;

	getAverageNeigborSize(cloud->size()/2);


}


int dataManager::openPointCloud(string path)
{
	return pcl::io::loadPCDFile(path, *rawCloud);
}

void dataManager::onePointContraction(vector<int>& neighborPoints)
{
	//获取局部邻域点云
	pcl::PointCloud<PointT>::Ptr localCloud(new pcl::PointCloud<PointT>);
	for (auto i : neighborPoints)
	{
		localCloud->points.emplace_back((*cloud)[i]);
	}
	//计算邻域中心
	pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
	feature_extractor.setInputCloud(localCloud);
	feature_extractor.compute();

	Eigen::Vector3f mass_center;
	feature_extractor.getMassCenter(mass_center);

	//对邻域中的每个点进行初步收缩，对应到原始点云中
	for (int i = 0; i < neighborPoints.size(); i++)
	{
		PointT localPoint = (*cloud)[neighborPoints[i]];
		Eigen::Vector3f displacementVector(mass_center[0] - localPoint.x, mass_center[1] - localPoint.y, mass_center[2] - localPoint.z);
		PointT newLocalPoint;
		float enlarge = 0.2;
		newLocalPoint.x = localPoint.x + displacementVector[0] * enlarge;
		newLocalPoint.y = localPoint.y + displacementVector[1] * enlarge;
		newLocalPoint.z = localPoint.z + displacementVector[2] * enlarge;
		newLocalPoint.r = 255;
		//newLocalPoint.g = 255;
		//newLocalPoint.b = 255;
		float newLength = pow((newLocalPoint.x-searchPoint.x),2) 
			+ pow((newLocalPoint.y - searchPoint.y),2) + pow((newLocalPoint.z - searchPoint.z),2);
		if (newLength < PCARadiusSize)
		{
			newLocalPoint.a = 0.3;
		}
		(*cloud)[neighborPoints[i]] = newLocalPoint;
		
	}
}

void dataManager::onePointContraction(vector<int>& neighborPoints, vector<int>& pointIdxRadiusSearch)
{
	//获取PCA局部邻域点云
	pcl::PointCloud<PointT>::Ptr localCloud(new pcl::PointCloud<PointT>);
	for (auto i : pointIdxRadiusSearch)
	{
		localCloud->points.emplace_back((*cloud)[i]);
	}
	//计算邻域中心
	pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
	feature_extractor.setInputCloud(localCloud);
	feature_extractor.compute();

	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center; 
	
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	float principal_Directivity = major_value / (major_value + middle_value + minor_value);
	if (principal_Directivity > linearity)
	{
		finishedContractionPoints.emplace(cloudIteration);
		return;
	}
	//计算KNN
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

	field_move_vector = vectorNorm_local_displacement_vector * cosANGLE*(1 - principal_Directivity)*major_vector
		+ (local_displacement_vector - vectorNorm_local_displacement_vector * cosANGLE * major_vector);

	//计算移动后的新点
	searchPoint.x = searchPoint.x + field_move_vector[0];
	searchPoint.y = searchPoint.y + field_move_vector[1];
	searchPoint.z = searchPoint.z + field_move_vector[2];
	searchPoint.r = 255;
	//newLocalPoint.g = 255;
	//newLocalPoint.b = 255;

	(*cloud)[cloudIteration] = searchPoint;

}

void dataManager::pointsContraction(int pointNeighborSize)
{
	//如果已经达到收缩要求，在本次和之后的迭代中停止收缩
	if (finishedContractionPoints.count(cloudIteration) == 0)
	{
		pcl::KdTreeFLANN<PointT> kdtree;
		kdtree.setInputCloud(cloud);
		getAverageNeigborSize(cloudIteration, KNNcontraction_K);
		int K = KNNcontraction_K;
		//K个邻近点的平均距离的三倍范围内的点 用于计算PCA主向量
		int radius = PCARadiusSize * 3;
		searchPoint = (*cloud)[cloudIteration];

		std::vector<int> pointIdxKNNSearch(K);
		std::vector<float> pointKNNSquaredDistance(K);
		kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);


		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

		onePointContraction(pointIdxKNNSearch, pointIdxRadiusSearch);
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

void dataManager::pointsContraction()
{
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(cloud);
	int K = KNNcontraction_K;
	int radius = downSamplingNeigborSize;
	searchPoint = (*cloud)[cloudIteration];

	std::vector<int> pointIdxKNNSearch(K);
	std::vector<float> pointKNNSquaredDistance(K); 
	kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance);


	//std::vector<int> pointIdxRadiusSearch;
	//std::vector<float> pointRadiusSquaredDistance;
	//kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	onePointContraction(pointIdxKNNSearch);
	if(cloudIteration < cloud->size() - 1)
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
	if (cloud->points.size() < 50)
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
 	pcl::VoxelGrid<PointT> sor;    // 创建滤波（下采样）对象
	sor.setInputCloud(cloud);             //设置需要过滤的点云
	//设置滤波时创建的体素体积为xcm的立方体,
	sor.setLeafSize(filterSize, filterSize, filterSize);
	sor.filter(*cloud);          //执行滤波处理，储存输出点云
}


void dataManager::startVoxelTransform()
{
	pcl::copyPointCloud(*cloud, *voxelCloud);
	
	downSamplingNeighborPoints(voxelSize);
	
	//提取外接最小矩形，过于耗时
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
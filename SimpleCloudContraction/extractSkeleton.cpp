#include "extractSkeleton.h"

extractSkeleton::extractSkeleton()
{
	circleSize = 10;
	cloudIteration = 0;
	nodeDensity = 12;
	nodeIteration = 0;
}

void extractSkeleton::prepareSkeletonDataStructure(PointCloudT::Ptr acloud)
{
	isPointVisited.clear();
	isPointVisited.assign(acloud->size(), false);
	cloud = acloud;

	pcl::MomentOfInertiaEstimation <PointT> feature_extractor;
	feature_extractor.setInputCloud(cloud);
	feature_extractor.compute();
	feature_extractor.getAABB(min_point_AABB, max_point_AABB);

	circleSize = sqrt(pow(min_point_AABB.x - max_point_AABB.x, 2) +
		pow(min_point_AABB.y - max_point_AABB.y, 2) + pow(min_point_AABB.z - max_point_AABB.z, 2)) / nodeDensity;


	kdtree.setInputCloud(cloud);


}

int extractSkeleton::extractCurrentSkePoint()
{
	do
	{
		//如果队列为空
		if (nextNode.empty() == true)
		{
			if (cloudIteration >= cloud->size())
			{
				return -1;
			}
			//寻找下一个没有访问的点作为node搜索起点
			while (isPointVisited[cloudIteration] == true)
			{
				cloudIteration++;
				if (cloudIteration >= cloud->size())
				{
					return -1;
				}
			}
			isPointVisited[cloudIteration] = true;
			nextNode.emplace((*cloud)[cloudIteration]);
			nodeList.emplace_back((*cloud)[cloudIteration]);
		}
		searchPoint = nextNode.front();
		nextNode.pop();

		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		kdtree.radiusSearch(searchPoint, circleSize, pointIdxRadiusSearch, pointRadiusSquaredDistance);

		findNextNode(pointIdxRadiusSearch, pointRadiusSquaredDistance);

	} while (nextNode.empty() == false);
	return 0;
}

//找最远的点（在主特征向量方向上）作为下一个node节点
void extractSkeleton::findNextNode(vector<int>& pointIdxRadiusSearch, vector<float>& pointRadiusSquaredDistance)
{
	//主特征向量方向 同向和反向 都可能有下一个node节点
	unordered_map<int, float> distanceDic;
	unordered_map<int, float> distanceDicOpposition;
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
	Eigen::Vector3f displacement_vector;
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter(mass_center);
	for (int i = 0; i < pointIdxRadiusSearch.size(); i++)
	{
		if (isPointVisited[pointIdxRadiusSearch[i]] == true)
		{
			continue;
		}
		//判断是否在60度范围类
		PointT localPoint = (*cloud)[pointIdxRadiusSearch[i]];
		displacement_vector[0] = localPoint.x - mass_center[0];
		displacement_vector[1] = localPoint.y - mass_center[1];
		displacement_vector[2] = localPoint.z - mass_center[2];
		float result_Multi_Vector = displacement_vector[0] * major_vector[0]
			+ displacement_vector[1] * major_vector[1] + displacement_vector[2] * major_vector[2];
		float vectorNorm__displacement_vector = sqrt(displacement_vector[0] * displacement_vector[0]
			+ displacement_vector[1] * displacement_vector[1] + displacement_vector[2] * displacement_vector[2]);
		float cosANGLE = result_Multi_Vector / vectorNorm__displacement_vector;

		if (cosANGLE >= sqrt(3) / 2)
		{
			distanceDic[pointIdxRadiusSearch[i]] = pointRadiusSquaredDistance[i];
		}
		else if (cosANGLE <= -sqrt(3) / 2)
		{
			distanceDicOpposition[pointIdxRadiusSearch[i]] = pointRadiusSquaredDistance[i];
		}
	}
	//找下一个node
	if (distanceDic.empty() == false)
	{
		int id = distanceDic.begin()->first;
		float maxdis = distanceDic.begin()->second;
		int secondid = id;
		float secondDis = maxdis;
		for (auto i = distanceDic.begin(); i != distanceDic.end(); i++)
		{
			if (i->second > maxdis)
			{
				secondDis = maxdis;
				secondid = id;
				maxdis = i->second;
				id = i->first;
			}
		}
		PointT nextNodePoint;
		nextNodePoint.x = ((*cloud)[id].x + (*cloud)[secondid].x) / 2;
		nextNodePoint.y = ((*cloud)[id].y + (*cloud)[secondid].y) / 2;
		nextNodePoint.z = ((*cloud)[id].z + (*cloud)[secondid].z) / 2;
		nextNodePoint.r = 255;
		nextNode.emplace(nextNodePoint);
		nodeList.emplace_back(nextNodePoint);
	}
	if (distanceDicOpposition.empty() == false)
	{
		int id = distanceDicOpposition.begin()->first;
		float maxdis = distanceDicOpposition.begin()->second;
		int secondid = id;
		float secondDis = maxdis;
		for (auto i = distanceDicOpposition.begin(); i != distanceDicOpposition.end(); i++)
		{
			if (i->second > maxdis)
			{
				secondDis = maxdis;
				secondid = id;
				maxdis = i->second;
				id = i->first;
			}
		}
		PointT nextNodePoint;
		nextNodePoint.x = ((*cloud)[id].x + (*cloud)[secondid].x) / 2;
		nextNodePoint.y = ((*cloud)[id].y + (*cloud)[secondid].y) / 2;
		nextNodePoint.z = ((*cloud)[id].z + (*cloud)[secondid].z) / 2;
		nextNodePoint.r = 255;
		nextNode.emplace(nextNodePoint);
		nodeList.emplace_back(nextNodePoint);
	}
	for (auto i : pointIdxRadiusSearch)
	{
		isPointVisited[i] = true;
	}

}

void extractSkeleton::connectSkeltonNode()
{
	localNodeCloud.reset(new PointCloudT);
	
	for (auto i : nodeList)
	{
		localNodeCloud->points.emplace_back(i);
	}
	kdtree.setInputCloud(localNodeCloud);

	PointT currPoint = (*localNodeCloud)[nodeIteration];
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	kdtree.radiusSearch(currPoint, circleSize, pointIdxRadiusSearch, pointRadiusSquaredDistance);

}

void extractSkeleton::prepareNodesDataStructure()
{

}
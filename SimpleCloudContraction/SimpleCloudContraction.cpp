
#pragma execution_character_set("utf-8")
#include "SimpleCloudContraction.h"

SimpleCloudContraction::SimpleCloudContraction(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);
	//点云初始化
	cloudManager = new dataManager(); 
	cloudManager->KNNcontraction_K = ui.setNeighborSizeLineEdit->text().toInt();
	
	skeletonManager = new extractSkeleton();

	//点云ui界面元素绑定
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	ui.screen->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.screen->GetInteractor(), ui.screen->GetRenderWindow());
	//坐标轴
	//viewer->addCoordinateSystem(500);
	pointCloudCreateShow();
	updatePointCloudSizeSlot();

	QObject::connect(ui.nextFrameButton, SIGNAL(clicked()), this, SLOT(openNextFrameSlot()));
	QObject::connect(ui.recoverCloudButton, SIGNAL(clicked()), this, SLOT(setRawPointCloudSlot()));
	QObject::connect(ui.openPointCloudButton, SIGNAL(clicked()), this, SLOT(openPointCloudSlot()));
	QObject::connect(ui.removeNeighborButton, SIGNAL(clicked()), this, SLOT(removeNeighborPointsSlot()));
	QObject::connect(ui.downSamplingButton, SIGNAL(clicked()), this, SLOT(downSamplingButtonSlot()));
	QObject::connect(ui.updatePointCloudSizeButton, SIGNAL(clicked()), this, SLOT(updatePointCloudSizeSlot()));
	QObject::connect(ui.savePCDFileButton, SIGNAL(clicked()), this, SLOT(savePCDSlot()));
	QObject::connect(ui.startVoxelTransformButton, SIGNAL(clicked()), this, SLOT(startVoxelTransformSlot()));
	QObject::connect(ui.resetVoxelButton, SIGNAL(clicked()), this, SLOT(resetVoxelSlot()));
	QObject::connect(ui.extractCurrentNodeButton, SIGNAL(clicked()), this, SLOT(extractCurrentNodeSlot()));
	QObject::connect(ui.resetNodeButton, SIGNAL(clicked()), this, SLOT(resetNodePointSlot()));
	QObject::connect(ui.connectNodesButton, SIGNAL(clicked()), this, SLOT(connectNodesSlot()));
	QObject::connect(ui.resetConnectedLinesButton, SIGNAL(clicked()), this, SLOT(resetConnectLineSlot()));
	QObject::connect(ui.searchAndConnectButton, SIGNAL(clicked()), this, SLOT(searchNodeConnectLineSlot()));

	
	
	QObject::connect(ui.setNeighborSizeLineEdit, SIGNAL(editingFinished()), this, SLOT(setNeighborSizeSlot()));
	QObject::connect(ui.downSamplingSizeLineEdit, SIGNAL(editingFinished()), this, SLOT(setDownSamplingSizeSlot()));
	QObject::connect(ui.linearityLineEdit, SIGNAL(editingFinished()), this, SLOT(setLinearitySlot()));
	QObject::connect(ui.voxelSizeLineEdit, SIGNAL(editingFinished()), this, SLOT(setVoxelSizeSlot()));
	QObject::connect(ui.nodeDensityLineEdit, SIGNAL(editingFinished()), this, SLOT(setNodeDensitySlot()));
	

	setNeighborSizeSlot();
	setDownSamplingSizeSlot();
	setLinearitySlot();
	setVoxelSizeSlot();
	setNodeDensitySlot();

}

void SimpleCloudContraction::openPointCloudSlot()
{
	QString type_pcd = "PCD(*.pcd)";
	QString fileName = QFileDialog::getOpenFileName(this, "请选择点云", ui.pointCloudFilePathLineEdit->text(), type_pcd);
	if (fileName == "")
	{
		QMessageBox::warning(this, "Error", "没有选中点云！");
		return;
	}
	//防止出现中文乱码
	string pcdFileName = string(fileName.toLocal8Bit());
	if(cloudManager->openPointCloud(pcdFileName) != 0)
	{
		QMessageBox::warning(this, "Error", "读取失败！");
		return;
	}
	//默认下采样一次
	//cloudManager->downSamplingNeighborPoints(120, cloudManager->rawCloud);
	setRawPointCloudSlot();

	ui.setNeighborSizeLineEdit->setText(QString::number(10));
	setNeighborSizeSlot();
	cloudManager->resetParameters();
}

void SimpleCloudContraction::savePCDSlot()
{

	QString savepath = QFileDialog::getSaveFileName(this, tr("保存图片"), 
		ui.pointCloudFilePathLineEdit->text(), "PointClouldData(*.pcd)");
	string pcdName = string(savepath.toLocal8Bit());
	//点云保存
	cloudManager->cloud->height = 1;
	cloudManager->cloud->width = cloudManager->cloud->size();
	if (pcl::io::savePCDFileBinary(pcdName, *(cloudManager->cloud)))
	{
		QMessageBox::information(NULL, "提示", "未完成保存");
	}
	else
	{
		QMessageBox::information(NULL, "提示", "已完成保存");
	}

}

void SimpleCloudContraction::pointCloudCreateShow()
{
	viewer->removePointCloud("cloud");
	viewer->addPointCloud(cloudManager->cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	ui.screen->repaint();
}

//点云界面更新
void SimpleCloudContraction::showPointCloud()
{
	viewer->updatePointCloud(cloudManager->cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	ui.screen->repaint();

}

void SimpleCloudContraction::showPointCloud(PointCloudT::Ptr cloud)
{
	viewer->updatePointCloud(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	ui.screen->repaint();

}

//进行一次全局所有点的搜索
void SimpleCloudContraction::openNextFrameSlot()
{
	int pointCloudSize = cloudManager->cloud->points.size();
	cloudManager->principal_Directivity.resize(pointCloudSize, 0);
	for (int i = 0; i < pointCloudSize; i++)
	{
		cloudManager->pointsContraction();
		showPointCloud();
		ui.textBrowser->append(QString::number(i)
			+ " , 线性度：" + QString::number(cloudManager->principal_Directivity[i]).left(5));
		ui.textBrowser->repaint();

		QString PCARadiusSize = "PCA半径：" + QString::number(cloudManager->PCARadiusSize * 3);
		ui.PCARadiusSize->setText(PCARadiusSize);

		QString pointsLeftForContraction = "待收缩点数：" 
			+ QString::number(cloudManager->cloud->size() - cloudManager->finishedContractionPoints.size());
		ui.pointsLeftForContraction->setText(pointsLeftForContraction);
	}

	showPointCloud();

	ui.setNeighborSizeLineEdit->setText(QString::number(ui.setNeighborSizeLineEdit->text().toInt() + 10));
	setNeighborSizeSlot();
}

//复位按钮
void SimpleCloudContraction::setRawPointCloudSlot()
{
	copyPointCloud(*(cloudManager->rawCloud), *(cloudManager->cloud));
	updatePointCloudSizeSlot();
	pointCloudCreateShow();
}

void SimpleCloudContraction::setNeighborSizeSlot()
{
	cloudManager->KNNcontraction_K = ui.setNeighborSizeLineEdit->text().toInt();
}

void SimpleCloudContraction::removeNeighborPointsSlot()
{
	cloudManager->removeCloseAroundNeighborPoints();
	pointCloudCreateShow();
	updatePointCloudSizeSlot();
}

void SimpleCloudContraction::downSamplingButtonSlot()
{
	cloudManager->downSamplingNeighborPoints(cloudManager->downSamplingNeigborSize);
	updatePointCloudSizeSlot();
	pointCloudCreateShow();
}

void SimpleCloudContraction::setDownSamplingSizeSlot()
{
	cloudManager->downSamplingNeigborSize = ui.downSamplingSizeLineEdit->text().toInt();
}

void SimpleCloudContraction::updatePointCloudSizeSlot()
{
	QString pointSize = "点云大小:" + QString::number(cloudManager->cloud->size());
	ui.pointCloudSize->setText(pointSize);
}

void SimpleCloudContraction::setLinearitySlot()
{
	cloudManager->linearity = ui.linearityLineEdit->text().toFloat();
}

void SimpleCloudContraction::setVoxelSizeSlot()
{
	cloudManager->voxelSize = ui.voxelSizeLineEdit->text().toInt();
}

void SimpleCloudContraction::startVoxelTransformSlot()
{
	cloudManager->startVoxelTransform();

	viewer->addCube(cloudManager->min_point_AABB.x, cloudManager->max_point_AABB.x, 
		cloudManager->min_point_AABB.y, cloudManager->max_point_AABB.y, 
		cloudManager->min_point_AABB.z, cloudManager->max_point_AABB.z,	1.0, 1.0, 1.0, "AABB");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
		pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
	Eigen::Quaternionf rotation(1, 0, 0, 0);
	for (int i = 0; i < cloudManager->voxelCenter.size(); i++)
	{
		string cube = "AABB" + to_string(i);
		viewer->addCube(cloudManager->voxelCenter[i], rotation, 
			cloudManager->voxelSize, cloudManager->voxelSize, cloudManager->voxelSize, cube);

		ui.screen->repaint();
	}
	showPointCloud();

}

void SimpleCloudContraction::resetVoxelSlot()
{
	copyPointCloud(*(cloudManager->voxelCloud), *(cloudManager->cloud));
	for (int i = 0; i < cloudManager->voxelCenter.size(); i++)
	{
		string cube = "AABB" + to_string(i);
		viewer->removeShape(cube);
	}
	viewer->removeShape("AABB");
	pointCloudCreateShow();
}

void SimpleCloudContraction::extractCurrentNodeSlot()
{
	//准备数据结构
	skeletonManager->prepareExtractNodePoint(cloudManager->cloud,  cloudManager);
	double radius = skeletonManager->circleSize / 6;

	while (skeletonManager->extractCurrentNodePoint() >= 0)
	{}
	//skeletonManager->removeDuplicatedNode();

	for (int nodeIndice = 0; nodeIndice < skeletonManager->nodeList.size(); nodeIndice++)
	{
		string sphere = "sphere" + to_string(nodeIndice);
		float linearity1 = skeletonManager->nodeListLinearity[nodeIndice];
		float lineartiy2 = skeletonManager->nodeListNeighborAverageLinearity[nodeIndice];
		if ((linearity1 > 0.75) || (lineartiy2 > 0.8) )
		{
			viewer->addSphere(skeletonManager->nodeList[nodeIndice], radius, 255, 0, 0, sphere);
			
		}
		else
		{
			viewer->addSphere(skeletonManager->nodeList[nodeIndice], radius, 0, 255, 0, sphere);

		}
		ui.transformStatusLabel->setText(QString::fromStdString(sphere + "已绘制"));
		ui.skeleTextBrowser->append(QString::fromStdString("Node" + to_string(nodeIndice) 
			+ "线性度：" + to_string(skeletonManager->nodeListLinearity[nodeIndice]).substr(0,5) 
			+ " 邻域线性度：" + to_string(skeletonManager->nodeListNeighborAverageLinearity[nodeIndice]).substr(0, 5)));
		PointT textPoint;
		textPoint  = skeletonManager->nodeList[nodeIndice];
		textPoint.x += 2*radius;

		viewer->addText3D(to_string(nodeIndice), textPoint, 40, 0, 0, 1, "text" + to_string(nodeIndice));

		ui.transformStatusLabel->repaint();
		ui.skeleTextBrowser->repaint();
		ui.screen->repaint();

	}
	
	ui.transformStatusLabel->setText(QString::fromStdString("已全部绘制"+to_string(skeletonManager->nodeList.size())+ "个节点"));
	ui.transformStatusLabel->repaint();
	
}

void SimpleCloudContraction::resetNodePointSlot()
{
	for (int nodeIndice = 0; nodeIndice < skeletonManager->nodeList.size(); nodeIndice++)
	{
		viewer->removeShape("sphere" + to_string(nodeIndice));
		viewer->removeShape("text" + to_string(nodeIndice));
	}
	skeletonManager->resetParameter();
	
	ui.screen->repaint();
	ui.transformStatusLabel->setText("已恢复初始状态");
}

void SimpleCloudContraction::setNodeDensitySlot()
{
	skeletonManager->nodeDensity = ui.nodeDensityLineEdit->text().toInt();
}

void SimpleCloudContraction::connectNodesSlot()
{
	lineCount = 0;
	string lineName;
	skeletonManager->connectSkeltonNode(skeletonManager->circleSize);
	for (int i = 0; i < skeletonManager->nodeList.size(); i++)
	{
		PointT startPoint, endPoint;

		startPoint = (*skeletonManager->localNodeCloud)[skeletonManager->skeletonGraph->adjList[i]->vertex];
		ArcNodePtr nodeptr = skeletonManager->skeletonGraph->adjList[i]->firstedge;
		while (nodeptr != nullptr)
		{
			lineCount++;
			lineName = "line" + to_string(lineCount);
			endPoint = (*skeletonManager->localNodeCloud)[nodeptr->adjvex];
			nodeptr = nodeptr->next;
			viewer->addLine(startPoint, endPoint, 0, 0, 1, lineName);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, lineName);
			ui.screen->repaint();
		}
	}
}

void SimpleCloudContraction::resetConnectLineSlot()
{
	for (; lineCount>0; lineCount--)
	{
		viewer->removeShape("line" + to_string(lineCount));
	}
	ui.screen->repaint();
	skeletonManager->skeletonGraph->adjList.clear();

}

void SimpleCloudContraction::searchNodeConnectLineSlot()
{

	skeletonManager->selectedNodeList.clear();
	//准备数据结构
	skeletonManager->prepareExtractNodePoint(cloudManager->cloud, cloudManager);
	double radius = skeletonManager->circleSize / 6;

	while (skeletonManager->extractCurrentNodePoint() >= 0){}

	skeletonManager->getLeftPointCloud();
	
	//身体部位点云二次收缩
	copyPointCloud((*cloudManager->cloud), (*cloudManager->lastFrameCloud));
	copyPointCloud((*skeletonManager->bodyCloud), (*cloudManager->cloud));
	cloudManager->KNNcontraction_K = 50;

	int pointCloudSize = cloudManager->cloud->points.size();
	cloudManager->resetParameters();
	for (int i = 0; i < pointCloudSize; i++)
	{
		cloudManager->pointsContraction();
		showPointCloud();
	}
	copyPointCloud((*cloudManager->lastFrameCloud), (*cloudManager->cloud));


	//准备数据结构
	skeletonManager->prepareExtractNodePoint(skeletonManager->bodyCloud, cloudManager);
	//还原之前的全局pca半径大小
	skeletonManager->circleSize = radius * 6 * 3;

	while (skeletonManager->extractCurrentNodePoint() >= 0)
	{}

	//计算邻域中心
	pcl::MomentOfInertiaEstimation<PointT> feature_extractor;
	feature_extractor.setInputCloud(skeletonManager->bodyCloud);
	feature_extractor.compute();

	Eigen::Vector3f mass_center;
	feature_extractor.getMassCenter(mass_center);
	
	
	//在节点列表中加入中心点
	PointT bodyCentralPoint;
	bodyCentralPoint.x = mass_center[0];
	bodyCentralPoint.y = mass_center[1];
	bodyCentralPoint.z = mass_center[2];
	skeletonManager->nodeList.emplace_back(bodyCentralPoint);
	for (auto i: skeletonManager->nodeList)
	{
		skeletonManager->selectedNodeList.emplace_back(i);

	}
	//去除太近的相邻节点
	/*for (int i = 0; i < skeletonManager->nodeList.size(); i++)
	{
		bool flag = false;
		PointT aPoint = skeletonManager->nodeList[i];
		PointT cPoint;
		for (int j = i + 1; j < skeletonManager->nodeList.size(); j++)
		{
			PointT bPoint = skeletonManager->nodeList[j];
			float distance = sqrt((aPoint.x - bPoint.x)*(aPoint.x - bPoint.x) +
				(aPoint.y - bPoint.y)*(aPoint.y - bPoint.y) + (aPoint.z - bPoint.z)*(aPoint.z - bPoint.z));
			if (distance < radius * 3)
			{
				cPoint = bPoint;
				flag = true;
				break;
			}
		}
		if (flag == false)
		{
			float distanceOfPointa = pow(aPoint.x - mass_center[0], 2) + pow(aPoint.y - mass_center[1], 2) + pow(aPoint.z - mass_center[2], 2);
			float distanceOfPointc = pow(cPoint.x - mass_center[0], 2) + pow(cPoint.y - mass_center[1], 2) + pow(cPoint.z - mass_center[2], 2);
			if (distanceOfPointa < distanceOfPointc)
			{
				skeletonManager->selectedNodeList.emplace_back(aPoint);
			}
		}
	}*/
	skeletonManager->nodeList.clear();
	skeletonManager->nodeList = skeletonManager->selectedNodeList;

	for (int nodeIndice = 0; nodeIndice < skeletonManager->nodeList.size(); nodeIndice++)
	{
		string sphere = "sphere" + to_string(nodeIndice);
		viewer->addSphere(skeletonManager->nodeList[nodeIndice], radius, 0, 255, 0, sphere);

		ui.transformStatusLabel->setText(QString::fromStdString(sphere + "已绘制"));
		ui.skeleTextBrowser->append(QString::fromStdString("Node" + to_string(nodeIndice)));
		PointT textPoint;
		textPoint = skeletonManager->nodeList[nodeIndice];
		textPoint.x += 2 * radius;

		viewer->addText3D(to_string(nodeIndice), textPoint, 40, 0, 0, 1, "text" + to_string(nodeIndice));

		ui.transformStatusLabel->repaint();
		ui.skeleTextBrowser->repaint();
		ui.screen->repaint();

	}

	ui.transformStatusLabel->setText(QString::fromStdString("已全部绘制" + to_string(skeletonManager->nodeList.size()) + "个节点"));
	ui.transformStatusLabel->repaint();
	/**/showPointCloud();
}

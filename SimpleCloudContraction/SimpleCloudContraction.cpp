
#pragma execution_character_set("utf-8")
#include "SimpleCloudContraction.h"

SimpleCloudContraction::SimpleCloudContraction(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);
	//���Ƴ�ʼ��
	cloudManager = new dataManager(); 
	cloudManager->KNNcontraction_K = ui.setNeighborSizeLineEdit->text().toInt();
	
	skeletonManager = new extractSkeleton();

	//����ui����Ԫ�ذ�
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	ui.screen->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.screen->GetInteractor(), ui.screen->GetRenderWindow());
	//������
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
	QObject::connect(ui.resetConnectedLinesButton, SIGNAL(clicked()), this, SLOT(resetNodeConnectLineSlot()));
	
	
	
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
	QString fileName = QFileDialog::getOpenFileName(this, "��ѡ�����", ui.pointCloudFilePathLineEdit->text(), type_pcd);
	if (fileName == "")
	{
		QMessageBox::warning(this, "Error", "û��ѡ�е��ƣ�");
		return;
	}
	//��ֹ������������
	string pcdFileName = string(fileName.toLocal8Bit());
	if(cloudManager->openPointCloud(pcdFileName) != 0)
	{
		QMessageBox::warning(this, "Error", "��ȡʧ�ܣ�");
		return;
	}

	setRawPointCloudSlot();
}


void SimpleCloudContraction::savePCDSlot()
{

	QString savepath = QFileDialog::getSaveFileName(this, tr("����ͼƬ"), 
		ui.pointCloudFilePathLineEdit->text(), "PointClouldData(*.pcd)");
	string pcdName = string(savepath.toLocal8Bit());
	//���Ʊ���
	cloudManager->cloud->height = 1;
	cloudManager->cloud->width = cloudManager->cloud->size();
	if (pcl::io::savePCDFileBinary(pcdName, *(cloudManager->cloud)))
	{
		QMessageBox::information(NULL, "��ʾ", "δ��ɱ���");
	}
	else
	{
		QMessageBox::information(NULL, "��ʾ", "����ɱ���");
	}

}


//�������ݸ���
//���룺c ����ָ��c
void SimpleCloudContraction::pointCloudUpdateSlot(PointCloudT::Ptr c)
{
	//�������ݸ���
	//cloud = c;
	showPointCloud();
}


void SimpleCloudContraction::pointCloudCreateShow()
{
	viewer->removePointCloud("cloud");
	viewer->addPointCloud(cloudManager->cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	ui.screen->repaint();
}

//���ƽ������
void SimpleCloudContraction::showPointCloud()
{
	viewer->updatePointCloud(cloudManager->cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	ui.screen->repaint();

}

//����һ��ȫ�����е������
void SimpleCloudContraction::openNextFrameSlot()
{
	int pointCloudSize = cloudManager->cloud->points.size();
	cloudManager->principal_Directivity.resize(pointCloudSize, 0);
	for (int i = 0; i < pointCloudSize; i++)
	{
		ui.textBrowser->append(QString::number(cloudManager->cloudIteration)
			+ " , ���Զȣ�" + QString::number(cloudManager->principal_Directivity[i]).left(5));
		ui.textBrowser->repaint();
		cloudManager->pointsContraction();
		showPointCloud();

		QString PCARadiusSize = "PCA�뾶��" + QString::number(cloudManager->PCARadiusSize * 3);
		ui.PCARadiusSize->setText(PCARadiusSize);

		QString pointsLeftForContraction = "������������" 
			+ QString::number(cloudManager->cloud->size() - cloudManager->finishedContractionPoints.size());
		ui.pointsLeftForContraction->setText(pointsLeftForContraction);
	}

	showPointCloud();

}

void SimpleCloudContraction::setRawPointCloudSlot()
{
	copyPointCloud(*(cloudManager->rawCloud), *(cloudManager->cloud));
	pointCloudCreateShow();
	updatePointCloudSizeSlot();
	cloudManager->finishedContractionPoints.clear();
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
	QString pointSize = "���ƴ�С:" + QString::number(cloudManager->cloud->size());
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
	//׼�����ݽṹ
	skeletonManager->prepareExtractNodePoint(cloudManager->cloud);
	double radius = skeletonManager->circleSize / 6;

	while (skeletonManager->extractCurrentNodePoint() >= 0)
	{}
	//skeletonManager->removeDuplicatedNode();

	for (int nodeIndice = 0; nodeIndice < skeletonManager->nodeList.size(); nodeIndice++)
	{
		string sphere = "sphere" + to_string(nodeIndice+1);
		viewer->addSphere(skeletonManager->nodeList[nodeIndice], radius, 0, 255, 0, sphere);
		ui.transformStatusLabel->setText(QString::fromStdString(sphere + "�ѻ���"));
		ui.transformStatusLabel->repaint();
		ui.screen->repaint();

	}
	
	ui.transformStatusLabel->setText(QString::fromStdString("��ȫ������"+to_string(skeletonManager->nodeList.size())+ "���ڵ�"));
	ui.transformStatusLabel->repaint();
	
}

void SimpleCloudContraction::resetNodePointSlot()
{
	for (int nodeIndice = 1; nodeIndice <= skeletonManager->nodeList.size(); nodeIndice++)
	{
		viewer->removeShape("sphere" + to_string(nodeIndice));
	}
	skeletonManager->cloudIteration = 0;
	skeletonManager->isPointVisited.assign(100, false);
	skeletonManager->nodeList.clear();
	skeletonManager->nodeIteration = 0;
	while (skeletonManager->nextNode.empty() == false)
	{
		skeletonManager->nextNode.pop();
	}
	
	ui.screen->repaint();
	ui.transformStatusLabel->setText("�ѻָ���ʼ״̬");
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

void SimpleCloudContraction::resetNodeConnectLineSlot()
{
	for (; lineCount>0; lineCount--)
	{
		viewer->removeShape("line" + to_string(lineCount));
	}
	ui.screen->repaint();
}
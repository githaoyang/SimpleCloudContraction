#pragma once

#include <QtWidgets/QWidget>
#include <qfile.h>
#include <qfiledialog.h> 
#include <qmessagebox.h>
#include "ui_SimpleCloudContraction.h"
#include "dataManage.h"
#include "extractSkeleton.h"

class SimpleCloudContraction : public QWidget
{
    Q_OBJECT

public:
    SimpleCloudContraction(QWidget *parent = Q_NULLPTR);



private:
    Ui::SimpleCloudContractionClass ui;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;	//PCL可视化窗口
	

	void showPointCloud();
	dataManager* cloudManager;
	extractSkeleton* skeletonManager;

	int nodeIndice = 0;

private slots:
	void openNextFrameSlot();
	void pointCloudUpdateSlot(PointCloudT::Ptr c);
	void pointCloudCreateShow();
	void setNeighborSizeSlot();
	void setRawPointCloudSlot();
	void openPointCloudSlot();
	void removeNeighborPointsSlot();
	void downSamplingButtonSlot();
	void setDownSamplingSizeSlot();
	void updatePointCloudSizeSlot();
	void setLinearitySlot();
	void savePCDSlot();
	void startVoxelTransformSlot();
	void setVoxelSizeSlot();
	void resetVoxelSlot();
	void extractCurrentSkePointSlot();
	void resetSkeletonPointSlot();
	void setNodeDensitySlot();
	void connectNodesSlot();
};

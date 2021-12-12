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
	void showPointCloud(PointCloudT::Ptr cloud);
	dataManager* cloudManager;
	extractSkeleton* skeletonManager;

	int lineCount;


private slots:

	void openNextFrameSlot();
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
	void extractCurrentNodeSlot();
	void resetNodePointSlot();
	void setNodeDensitySlot();
	void connectNodesSlot();
	void resetConnectLineSlot();
	void searchNodeConnectLineSlot();

};

#include "processplan.h"
#include "ui_processplan.h"

ProcessPlan::ProcessPlan(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ProcessPlan),
    singleStep(0)
{
    ui->setupUi(this);
    connect(ui->btn_start,&QPushButton::clicked,this,&ProcessPlan::start);
    connect(ui->btn_clear,&QPushButton::clicked,this,&ProcessPlan::clear);
    connect(ui->btn_reset,&QPushButton::clicked,this,&ProcessPlan::reset);
    ui->cbox_pointcloud->setCurrentIndex(-1);
}
ProcessPlan::~ProcessPlan()
{
    delete ui;
}

void ProcessPlan::init()
{
    ui->treeLayout->addWidget(processTree);
    connect(ui->btn_delete,&QPushButton::clicked,processTree,&ProcessTree::clearItem);
    connect(ui->btn_save,&QPushButton::clicked,processTree,&ProcessTree::saveItems);
    connect(ui->btn_load,&QPushButton::clicked,processTree,&ProcessTree::loadItems);
    connect(ui->check_enable,&QCheckBox::clicked,processTree,&ProcessTree::processChanged);
    connect(processTree,&ProcessTree::captureState,this,&ProcessPlan::captureStateChanged);
    connect(processTree,&ProcessTree::captureCloud,this,&ProcessPlan::processCaptureCloud);
    processTree->processEnable=true;
}

void ProcessPlan::start()
{
    allProcess=processTree->getAllProcess();
    selectedClouds=cloudTree->getSelectedClouds();
    switch (ui->cbox_pointcloud->currentIndex()){
    case 0://selectcloud
        if(selectedClouds.size()<=0) {
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        if((singleStep==0)|ui->rbtn_looponce->isChecked()) {
            processedClouds.clear();
            processedClouds=selectedClouds;
        }
        if(ui->rbtn_single->isChecked()) {
            if(singleStep<int(allProcess.size())){
                QTreeWidgetItem *item=processTree->topLevelItem(singleStep);
                processTree->setCurrentItem(item);
                this->processCloud(processedClouds,allProcess,singleStep);
                singleStep++;
            }
            else {
                singleStep=0;
                console->info(tr("Press again to repeat the loop process."));
            }
        }
        else if(ui->rbtn_looponce->isChecked()) {
            processTree->setCurrentItem(nullptr);
            for (size_t i=0;i<allProcess.size();i++) {
                this->processCloud(processedClouds,allProcess,i);
            }
        }
        break;
    case 1://azurekinect
        captureCloud.id="k4a";
        emit processTree->deviceCapture();
        break;
    case 2://Zhisensor
        break;
    case -1:
        console->warning(tr("Please select a input pointcloud"));
        return;
    }
}

void ProcessPlan::reset()
{
    singleStep=0;
    cloudView->removeAllCloud();
    cloudView->removeAllShape();
    switch (ui->cbox_pointcloud->currentIndex()){
    case 0://selectcloud
        allClouds=cloudTree->getAllClouds();
        for(size_t i=0;i<allClouds.size();i++) {
            if(!cloudView->contains(allClouds[i].id)) {
                cloudView->updateCloud(allClouds[i].cloud,allClouds[i].id);
            }
        }
        break;
    case 1://azurekinect
        ui->btn_start->setText(tr("Start"));
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        cloudView->removeCloud("k4a");
        cloudView->removeCloud("device_cloud");
        break;
    case 2://photoneo
        break;
    case -1:
        console->warning(tr("Please select a input pointcloud"));
        return;
    }
}

void ProcessPlan::clear()
{
    processTree->clearAllItem();
    processedClouds.clear();
    this->reset();
}

void ProcessPlan::processCaptureCloud(const CloudXYZRGBN::Ptr &cloud)
{
    if((singleStep==0)|ui->rbtn_looponce->isChecked()) {
        cloudView->removeAllCloud();
        cloudView->removeAllShape();
        *captureCloud.cloud=*cloud;
        cloudView->updateCloud(captureCloud.cloud,captureCloud.id);
        processedClouds.clear();
        processedClouds.push_back(captureCloud);
    }
    allProcess=processTree->getAllProcess();
    if(ui->rbtn_single->isChecked()) {
        if(singleStep<int(allProcess.size())){
            QTreeWidgetItem *item=processTree->topLevelItem(singleStep);
            processTree->setCurrentItem(item);
            this->processCloud(processedClouds,allProcess,singleStep);
            singleStep++;
        }
        else {
            singleStep=0;
            console->info(tr("Press again to repeat the loop process."));
        }
    }
    else if(ui->rbtn_looponce->isChecked()) {
        processTree->setCurrentItem(nullptr);
        for (size_t i=0;i<allProcess.size();i++) {
            this->processCloud(processedClouds,allProcess,i);
        }
    }
}

void ProcessPlan::captureStateChanged(bool isAutoRefresh, bool isCapturing)
{
    if(isAutoRefresh) {
        if(!isCapturing) {
            ui->btn_start->setText(tr("Stop"));
            ui->btn_start->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
        } else{
            ui->btn_start->setText(tr("Start"));
            ui->btn_start->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        }
    }
}

void ProcessPlan::closeEvent(QCloseEvent *event)
{
    ui->treeLayout->removeWidget(processTree);
    processTree->setParent(nullptr);
    processTree->processEnable=false;
    processTree->clearAllItem();
    processedClouds.clear();
    cloudView->removeAllCloud();
    cloudView->removeAllShape();
    allClouds=cloudTree->getAllClouds();
    for(size_t i=0;i<allClouds.size();i++) {
        if(!cloudView->contains(allClouds[i].id)) {
            cloudView->updateCloud(allClouds[i].cloud,allClouds[i].id);
        }
    }
    cloudView->resetCamera();
}

void ProcessPlan::processCloud(std::vector<Cloud> &cloud, std::vector<Process> &p,const int &i)
{
    if(!p[i].isHidden)
        switch (p[i].type){
        case process_device:
            devcie.reset(new CloudXYZRGBN);
            *devcie=*cloud[0].cloud;
            cloudView->updateCloud(devcie,"device_cloud");
            break;
        case process_boundingBox:
            for (size_t j=0;j<cloud.size();j++){
                if(p[i].value[0]){
                    boundingbox=feature.BoundingBox_AABB(cloud[j].cloud);
                    cloudView->updateBoundingBox(boundingbox,"process_boundingBox_"+std::to_string(j),p[i].value[2],1,255,0,0);
                }
                else if(p[i].value[1]){
                    boundingbox=feature.BoundingBox_OBB(cloud[j].cloud);
                    cloudView->updateBoundingBox(boundingbox,"process_boundingBox_"+std::to_string(j),p[i].value[2],1,0,255,0);
                }
            }
            break;
        case process_color:
            for (size_t j=0;j<cloud.size();j++){
                if(p[i].str[0]==""){
                    common.setColor(cloud[j].cloud,p[i].value[0],p[i].value[1],p[i].value[2]);
                    cloudView->updateCloud(cloud[j].cloud,cloud[j].id);
                } else {
                    if(p[i].str[0]!="r") {
                        common.setColor(cloud[j].cloud,p[i].str[0]);
                        cloudView->updateCloud(cloud[j].cloud,cloud[j].id);
                    }
                    else {
                        common.setColor(cloud[j].cloud,rand()%256,rand()%256,rand()%256);
                        cloudView->updateCloud(cloud[j].cloud,cloud[j].id);
                    }
                }
            }
            break;
        case process_coordinate:
            for (size_t j=0;j<cloud.size();j++){
                if(p[i].value[0]) {
                    cloudView->updateCoord(p[i].value[1],Eigen::Transform<float, 3, Eigen::Affine>(cloud[j].box.transformation),"process_coord_"+std::to_string(j));
                } else if(p[i].value[2]) {
                    cloudView->setCloudSize(cloud[j].id,p[i].value[3]);
                }
            }
            break;
        case process_cloudFiter:
            for (size_t j=0;j<cloud.size();j++){
                switch (int(p[i].value[0])){
                case 0://PassThrough
                    cloud[j].cloud=filters.PassThrough(cloud[j].cloud,p[i].str[0],p[i].value[2],p[i].value[3],p[i].value[1]);
                    break;
                case 1://VoxelGrid
                    if(!p[i].value[2])
                        cloud[j].cloud=filters.VoxelGrid(cloud[j].cloud,p[i].value[3],p[i].value[4],p[i].value[5],p[i].value[6],p[i].value[1]);
                    else
                        cloud[j].cloud=filters.ApproximateVoxelGrid(cloud[j].cloud,p[i].value[3],p[i].value[4],p[i].value[5],p[i].value[6]);
                    break;
                case 2://StatisticalOutlierRemoval
                    cloud[j].cloud=filters.StatisticalOutlierRemoval(cloud[j].cloud,p[i].value[2],p[i].value[3],p[i].value[1]);
                    break;
                case 3://RadiusOutlierRemoval
                    cloud[j].cloud=filters.RadiusOutlierRemoval(cloud[j].cloud,p[i].value[2],p[i].value[3],p[i].value[1]);
                    break;
                case 4://ProjectInliers
                    selectedModel=modelTree->getSelectedModel();
                    cloud[j].cloud=filters.ProjectInliers(cloud[j].cloud,selectedModel,p[i].value[2],p[i].value[3]);
                    break;
                case 5:
                    cloud[j].cloud=filters.MovingLeastSquares(cloud[j].cloud,p[i].value[2],p[i].value[3],p[i].value[4],p[i].value[5],p[i].value[6],p[i].value[7],
                            p[i].value[8],p[i].value[9],p[i].value[10],p[i].value[11],p[i].value[12],p[i].value[13]);
                    break;
                }
                cloudView->updateCloud(cloud[j].cloud,cloud[j].id);
            }
            break;
        case process_normols:
            for (size_t j=0;j<cloud.size();j++){
                if(p[i].value[0])
                    viewpoint<<cloud[j].cloud->sensor_origin_.coeff(0),cloud[j].cloud->sensor_origin_.coeff(1),cloud[j].cloud->sensor_origin_.coeff(2);
                else if(p[i].value[1])
                    viewpoint=cloud[j].center();
                else if(p[i].value[2])
                    viewpoint<<std::numeric_limits<float>::max (),std::numeric_limits<float>::max (),std::numeric_limits<float>::max ();
                if(p[i].value[3])
                    cloud[j].cloud=feature.NormalEstimation(cloud[j].cloud,int(p[i].value[4]),viewpoint);
                else if(p[i].value[5])
                    cloud[j].cloud=feature.NormalEstimation(cloud[j].cloud,p[i].value[6],viewpoint);
                if(p[i].value[7])
                    for(size_t j=0;j<cloud[j].cloud->points.size();j++){
                        cloud[j].cloud->points[j].normal_x=-cloud[j].cloud->points[j].normal_x;
                        cloud[j].cloud->points[j].normal_y=-cloud[j].cloud->points[j].normal_y;
                        cloud[j].cloud->points[j].normal_z=-cloud[j].cloud->points[j].normal_z;
                    }
                cloudView->updateCloud(cloud[j].cloud,cloud[j].id);
                if(p[i].value[8])
                    cloudView->updateNormols(cloud[j].cloud,p[i].value[9],p[i].value[10],"process_normols"+std::to_string(j));
            }
            break;
        case process_scale:
            for (size_t j=0;j<cloud.size();j++){
                if(p[i].value[0])
                    cloud[j].cloud=common.ScaleCloud(cloud[j].cloud,cloud[j].center(),p[i].value[3],p[i].value[4],p[i].value[5],true);
                else if(p[i].value[1])
                    cloud[j].cloud=common.ScaleCloud(cloud[j].cloud,cloud[j].center(),p[i].value[3],p[i].value[4],p[i].value[5],false);
                cloudView->updateCloud(cloud[j].cloud,cloud[j].id);
                if(p[i].value[2])
                    cloudView->resetCamera();
            }
            break;
        case process_segmentation:
            segmentedClouds.clear();
            for (size_t j=0;j<cloud.size();j++){
                switch (int(p[i].value[0])){
                case 0://SACSegmentation
                    if(!p[i].value[1])
                        segmentedClouds=seg.SACSegmentation(cloud[j].cloud,coefs,p[i].value[2],p[i].value[3],p[i].value[4],p[i].value[5],p[i].value[6],p[i].value[7],
                                p[i].value[8],p[i].value[9],p[i].value[10],p[i].value[11],p[i].value[12],p[i].value[13],p[i].value[14],p[i].value[15]);
                    else
                        segmentedClouds=seg.SACSegmentationFromNormals(cloud[j].cloud,coefs,p[i].value[2],p[i].value[3],p[i].value[4],p[i].value[5],p[i].value[6],p[i].value[7],
                                p[i].value[8],p[i].value[9],p[i].value[10],p[i].value[11],p[i].value[12],p[i].value[13],p[i].value[14],p[i].value[15],p[i].value[16],
                                p[i].value[17],p[i].value[18],p[i].value[19]);
                    break;
                case 1://EuclideanClusterExtraction
                    segmentedClouds=seg.EuclideanClusterExtraction(cloud[j].cloud,p[i].value[1],p[i].value[2],p[i].value[3]);
                    break;
                case 2://RegionGrowing
                    if(!p[i].value[1])
                        segmentedClouds=seg.RegionGrowing(cloud[j].cloud,p[i].value[2],p[i].value[3],p[i].value[4],p[i].value[5],p[i].value[6],p[i].value[7],
                                p[i].value[8],p[i].value[9],p[i].value[10]);
                    else
                        segmentedClouds=seg.RegionGrowingRGB(cloud[j].cloud,p[i].value[2],p[i].value[3],p[i].value[4],p[i].value[5],p[i].value[6],p[i].value[7],
                                p[i].value[8],p[i].value[9],p[i].value[10],p[i].value[11],p[i].value[12],p[i].value[13],p[i].value[14]);
                    break;
                case 3://MinCutSegmentation
                    segmentedClouds=seg.MinCutSegmentation(cloud[j].cloud,cloud[j].center(),p[i].value[1],p[i].value[2],p[i].value[3],p[i].value[4]);
                    break;
                case 4://DonSegmentation
                    segmentedClouds=seg.DonSegmentation(cloud[j].cloud,cloud[j].meanRadius(),p[i].value[1],p[i].value[2],p[i].value[3],p[i].value[4],p[i].value[5],p[i].value[6]);
                    break;
                case 5://SupervoxelClustering
                    segmentedClouds=seg.SupervoxelClustering(cloud[j].cloud,p[i].value[1],p[i].value[2],p[i].value[3],p[i].value[4],p[i].value[5],p[i].value[6]);
                    break;
                case 6://MorphologicalFilter
                    segmentedClouds=seg.MorphologicalFilter(cloud[j].cloud,p[i].value[1],p[i].value[2],p[i].value[3],p[i].value[4],p[i].value[5],p[i].value[6],p[i].value[7],p[i].value[8]);
                }
                cloudView->removeCloud(cloud[j].id);
            }
            cloud.clear();
            for(size_t n=0;n<segmentedClouds.size();n++){
                Cloud segCloud;
                *segCloud.cloud=*segmentedClouds[n];
                segCloud.id="process_cloud_"+std::to_string(n);
                cloud.push_back(segCloud);
                cloudView->updateCloud(segCloud.cloud,segCloud.id);
            }
            break;
        case process_surface:
            for (size_t j=0;j<cloud.size();j++){
                switch (int(p[i].value[0])) {
                case 0://GreedyProjectionTriangulation
                    triangle=sur.GreedyProjectionTriangulation(cloud[j].cloud,p[i].value[2],p[i].value[3],p[i].value[4],p[i].value[5],p[i].value[6],p[i].value[7],
                            p[i].value[8],p[i].value[9]);
                    break;
                case 1://GridProjection
                    triangle=sur.GridProjection(cloud[j].cloud,p[i].value[2],p[i].value[3],p[i].value[4],p[i].value[5]);
                    break;
                case 2://Poisson
                    triangle=sur.Poisson(cloud[j].cloud,p[i].value[2],p[i].value[3],p[i].value[4],p[i].value[5],p[i].value[6],p[i].value[7],
                            p[i].value[8],p[i].value[9],p[i].value[10],p[i].value[11],p[i].value[12]);
                    break;
                }
                if(p[i].value[1])
                    cloudView->updateMesh(triangle,"process_surface_"+std::to_string(j));
                else
                    cloudView->updateMeshLine(triangle,"process_surface_"+std::to_string(j));
            }
            break;
        case process_transform:
            for (size_t j=0;j<cloud.size();j++){
                pcl::transformPointCloud(*cloud[j].cloud,*cloud[j].cloud,p[i].trans);
                cloudView->updateCloud(cloud[j].cloud,cloud[j].id);
            }
            break;
        case process_treesearch:
            for (size_t j=0;j<cloud.size();j++){
                if(p[i].value[0])  {
                    if(p[i].value[2])
                        cloud[j].cloud=tree.KDTree(cloud[j].cloud,p[i].value[5],int(p[i].value[6]));
                    else if(p[i].value[3])
                        cloud[j].cloud=tree.KDTree(cloud[j].cloud,p[i].value[5],p[i].value[7]);
                } else if(p[i].value[1]) {
                    if(p[i].value[2])
                        cloud[j].cloud=tree.OCTree(cloud[j].cloud,p[i].value[5],p[i].value[8],int(p[i].value[6]));
                    else if(p[i].value[3])
                        cloud[j].cloud=tree.OCTree(cloud[j].cloud,p[i].value[5],p[i].value[8],p[i].value[7]);
                    else if(p[i].value[4])
                        cloud[j].cloud=tree.OCTree(cloud[j].cloud,p[i].value[5],p[i].value[8]);
                }
                cloudView->updateCloud(cloud[j].cloud,cloud[j].id);
            }
            break;
        case process_keypoint:
            break;
        }
}

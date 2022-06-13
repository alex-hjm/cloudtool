#include "segmentation.h"
#include "ui_segmentation.h"

Segmentation::Segmentation(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Segmentation),
    modelType(pcl::SACMODEL_PLANE)
{
    ui->setupUi(this);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Segmentation::preview);
    connect(ui->btn_add,&QPushButton::clicked,this,&Segmentation::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Segmentation::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Segmentation::reset);
    connect(ui->cbox_segmentations,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        if((index==0)|(index==6))
            ui->check_negative->setEnabled(true);
        else {
            ui->check_negative->setChecked(false);
            ui->check_negative->setEnabled(false);
        }
    });
    ui->cbox_segmentations->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
    //SACSegmentation
    connect(ui->checkbox_fromNormals,&QCheckBox::clicked,[=](bool state){
        if(state){
            ui->dspin_distanceWeight->setEnabled(true);
        }
        else{
            ui->dspin_distanceWeight->setEnabled(false);
        }
    });
    //RegionGrowing
    connect(ui->check_smoothmode,&QCheckBox::clicked,[=](bool state){
        if(state) ui->dspin_smooth->setEnabled(true);
        else ui->dspin_smooth->setEnabled(false);
    });
    connect(ui->check_curvaturetest,&QCheckBox::clicked,[=](bool state){
        if(state) ui->dspin_curvature->setEnabled(true);
        else ui->dspin_curvature->setEnabled(false);
    });
    connect(ui->check_residualtest,&QCheckBox::clicked,[=](bool state){
        if(state) ui->dspin_residual->setEnabled(true);
        else ui->dspin_residual->setEnabled(false);
    });
}

Segmentation::~Segmentation()
{
    delete ui;
}

void Segmentation::init()
{
    cloudTree->setExtendedSelection(false);
}

void Segmentation::preview()
{
    Cloud selectedCloud=cloudTree->getSelectedCloud();
    if(selectedCloud.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    int currentIndex=ui->cbox_segmentations->currentIndex();
    for(size_t i=0;i<segmentedClouds.size();i++) {
        cloudView->removeCloud("segmented_"+std::to_string(i));
    }
    segmentedClouds.clear();
    switch (currentIndex)
    {
    case 0://SACSegmentation
        if(!ui->checkbox_fromNormals->isChecked()){
            segmentedClouds=seg.SACSegmentation(selectedCloud.cloud,coefs,ui->cbox_modelType->currentIndex(),ui->cbox_methodType->currentIndex(),
                                                ui->dspin_Threshold->value(),ui->dspin_Iterations->value(),ui->dspin_minRadius->value(),
                                                ui->dspin_maxRadius->value(),ui->check_optimize->isChecked(),ui->check_negative->isChecked());
            cloudView->showInfoText("SACSegmentation",30,"info");
        }
        else{
            segmentedClouds=seg.SACSegmentationFromNormals(selectedCloud.cloud,coefs,ui->cbox_modelType->currentIndex(),ui->cbox_methodType->currentIndex(),
                                                           ui->dspin_Threshold->value(),ui->dspin_Iterations->value(),ui->dspin_minRadius->value(),
                                                           ui->dspin_maxRadius->value(),ui->check_optimize->isChecked(),ui->check_negative->isChecked(),
                                                           ui->dspin_distanceWeight->value());
            cloudView->showInfoText("SACSegmentationFromNormals",30,"info");
        }
        break;
    case 1://EuclideanClusterExtraction
        segmentedClouds=seg.EuclideanClusterExtraction(selectedCloud.cloud,ui->dspin_tolerance->value(),ui->spin_min_cluster_size->value(),
                                                       ui->spin_max_cluster_size->value());
        cloudView->showInfoText("EuclideanClusterExtraction",30,"info");
        break;
    case 2://RegionGrowing
        segmentedClouds=seg.RegionGrowing(selectedCloud.cloud,ui->spin_minclustersize->value(),ui->spin_maxclustersize->value(),ui->check_smoothmode->isChecked(),
                                          ui->check_curvaturetest->isChecked(),ui->check_residualtest->isChecked(),ui->dspin_smooth->value(),
                                          ui->dspin_residual->value(),ui->dspin_curvature->value(),ui->spin_numofnei->value());
        cloudView->showInfoText("RegionGrowing",30,"info");
         break;
    case 3://RegionGrowingRGB
        segmentedClouds=seg.RegionGrowingRGB(selectedCloud.cloud,ui->spin_minclustersize->value(),ui->spin_maxclustersize->value(),ui->check_smoothmode->isChecked(),
                                             ui->check_curvaturetest->isChecked(),ui->check_residualtest->isChecked(),ui->dspin_smooth->value(),
                                             ui->dspin_residual->value(),ui->dspin_curvature->value(),ui->spin_numofnei->value(),ui->dspin_pointcolor->value(),
                                             ui->dspin_regioncolor->value(),ui->dspin_distance->value(),ui->spin_nghbr_number->value());
        cloudView->showInfoText("RegionGrowingRGB",30,"info");
        break;
    case 4://MinCutSegmentation
        segmentedClouds=seg.MinCutSegmentation(selectedCloud.cloud,selectedCloud.center(),ui->dspin_sigma->value(),ui->dspin_radius->value(),ui->dspin_weight->value(),
                                               ui->spin_neighbour_number->value());
        cloudView->showInfoText("MinCutSegmentation",30,"info");
        break;
    case 5://DonSegmentation
        if(ui->dspin_smallscale->value()>ui->dspin_largescale->value()){
            console->warning(tr("Please set the correct paramters!"));
            return;
        }
        segmentedClouds=seg.DonSegmentation(selectedCloud.cloud,selectedCloud.resolution,ui->dspin_smallscale->value(),ui->dspin_largescale->value(),ui->dspin_threshold->value(),
                                            ui->dspin_segradius->value(),ui->spin_mincluster->value(),ui->spin_maxcluster->value());
        cloudView->showInfoText("DonSegmentation",30,"info");
        break;
    case 6://SupervoxelClustering
        segmentedClouds=seg.SupervoxelClustering(selectedCloud.cloud,ui->dspin_voxelresolution->value(),ui->dspin_seedresolution->value(),
                                                 ui->dspin_colorimportance->value(),ui->dspin_spatialmportance->value(),ui->dspin_normallmportance->value(),
                                                 ui->check_transform->isChecked());
        cloudView->showInfoText("SupervoxelClustering",30,"info");
        break;
    case 7://MorphologicalFilter
        segmentedClouds=seg.MorphologicalFilter(selectedCloud.cloud,ui->spin_maxwinsize->value(),ui->dspin_slope->value(),ui->dspin_maxdistance->value(),
                                                ui->dspin_initialdistance->value(),ui->dspin_cellsize->value(),ui->dspin_base->value(),ui->check_negative->isChecked());
        cloudView->showInfoText("MorphologicalFilter",30,"info");
        break;
    }
    console->info(tr("The pointcloud has been segmented to %1 clouds,take time %2 ms").arg(segmentedClouds.size()).arg(seg.tocTime));
    if(segmentedClouds.size()<=0)return;
    for (size_t i=0;i<segmentedClouds.size();i++){
        if(segmentedClouds[i]->points.size()<=0)continue;
        cloudView->updateCloud(segmentedClouds[i],"segmented_"+std::to_string(i));
        cloudView->setCloudColor(segmentedClouds[i],"segmented_"+std::to_string(i),rand()%256,rand()%256,rand()%256);
        cloudView->setCloudSize("segmented_"+std::to_string(i),3);
    }
}

void Segmentation::add()
{
    Cloud selectedCloud=cloudTree->getSelectedCloud();
    if(selectedCloud.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(segmentedClouds.size()<=0) {
        console->warning(tr("Please segment the pointcloud!"));
        return;
    }
    for(size_t i=0;i<segmentedClouds.size();i++) {
        cloudView->removeCloud("segmented_"+std::to_string(i));
        Cloud cloud_segmented(segmentedClouds[i],selectedCloud.fileInfo);
        cloud_segmented.prefix("seg"+std::to_string(i)+"-");
        if(i==0)
            cloudTree->insertCloud(-1,cloud_segmented,true);
        else {
            int row=cloudTree->topLevelItemCount()-1;
            cloudTree->insertCloud(row,cloud_segmented,false);
        }
    }
    segmentedClouds.clear();
}

void Segmentation::apply()
{
    Cloud selectedCloud=cloudTree->getSelectedCloud();
    if(selectedCloud.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(segmentedClouds.size()<=0) {
        this->preview();
    }
    Index index=cloudTree->getSelectedIndex();
    for (size_t i=0;i<segmentedClouds.size();i++) {
        cloudView->removeCloud("segmented_"+std::to_string(i));
        Cloud cloud_segmented(segmentedClouds[i],selectedCloud.fileInfo);
        cloud_segmented.prefix("seg"+std::to_string(i)+"-");
        cloudTree->insertCloud(index.row,cloud_segmented,true);
    }
    cloudTree->clearCloud(index);
    segmentedClouds.clear();
}

void Segmentation::reset()
{
    cloudView->removeShape("info");
    if(segmentedClouds.size()<=0) return;
    for(size_t i=0;i<segmentedClouds.size();i++) {
        cloudView->removeCloud("segmented_"+std::to_string(i));
    }
    segmentedClouds.clear();
}

void Segmentation::closeEvent(QCloseEvent *event)
{
    this->reset();
    cloudTree->setExtendedSelection(true);
    return QWidget::closeEvent(event);
}

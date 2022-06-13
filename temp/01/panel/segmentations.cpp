#include "segmentations.h"
#include "ui_segmentations.h"

Segmentations::Segmentations(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::Segmentations),coefs(new ModelCoefficients)
{
    ui->setupUi(this);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Segmentations::preview);
    connect(ui->btn_add,&QPushButton::clicked,this,&Segmentations::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Segmentations::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Segmentations::reset);
    Segmentation *seg=new Segmentation;
    seg->moveToThread(&thread);
    qRegisterMetaType<Eigen::Vector3f>("Eigen::Vector3f &");
    qRegisterMetaType<Eigen::Vector3f>("Eigen::Vector3f");
    qRegisterMetaType<ModelCoefficients::Ptr>("ModelCoefficients::Ptr &");
    qRegisterMetaType<ModelCoefficients::Ptr>("ModelCoefficients::Ptr");
    qRegisterMetaType<std::vector<Cloud::Ptr>>("std::vector<Cloud::Ptr> &");
    qRegisterMetaType<std::vector<Cloud::Ptr>>("std::vector<Cloud::Ptr>");
    connect(&thread,&QThread::finished,seg,&QObject::deleteLater);
    connect(this,&Segmentations::SACSegmentation,seg,&Segmentation::SACSegmentation);
    connect(this,&Segmentations::SACSegmentationFromNormals,seg,&Segmentation::SACSegmentationFromNormals);
    connect(this,&Segmentations::EuclideanClusterExtraction,seg,&Segmentation::EuclideanClusterExtraction);
    connect(this,&Segmentations::RegionGrowing,seg,&Segmentation::RegionGrowing);
    connect(this,&Segmentations::RegionGrowingRGB,seg,&Segmentation::RegionGrowingRGB);
    connect(this,&Segmentations::MinCutSegmentation,seg,&Segmentation::MinCutSegmentation);
    connect(this,&Segmentations::DonSegmentation,seg,&Segmentation::DonSegmentation);
    connect(this,&Segmentations::MorphologicalFilter,seg,&Segmentation::MorphologicalFilter);
    connect(this,&Segmentations::SupervoxelClustering,seg,&Segmentation::SupervoxelClustering);
    connect(seg,&Segmentation::result,this,&Segmentations::result);
    thread.start();

    connect(ui->cbox_segmentations,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        if(index==0||index==6)
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

Segmentations::~Segmentations()
{
    thread.quit();
    thread.wait();
    delete ui;
}

void Segmentations::init(Console* &co,CloudView* &cv,CloudTree* &ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
    cloud_tree->setExtendedSelection(false);
}

void Segmentations::preview()
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    if(selectedCloud->empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    for(size_t i=0;i<segmented_clouds.size();i++)
        cloud_view->removeCloud("segmented_"+std::to_string(i));
    segmented_clouds.clear();
    switch (ui->cbox_segmentations->currentIndex())
    {
    case 0://SACSegmentation
        if(!ui->checkbox_fromNormals->isChecked()){
            emit SACSegmentation(selectedCloud,coefs,ui->cbox_modelType->currentIndex(),ui->cbox_methodType->currentIndex(),
                                                ui->dspin_Threshold->value(),ui->dspin_Iterations->value(),ui->dspin_minRadius->value(),
                                                ui->dspin_maxRadius->value(),ui->check_optimize->isChecked(),ui->check_negative->isChecked());
            cloud_view->showInfo("SACSegmentation",30,"info");
        }
        else{
            emit SACSegmentationFromNormals(selectedCloud,coefs,ui->cbox_modelType->currentIndex(),ui->cbox_methodType->currentIndex(),
                                                           ui->dspin_Threshold->value(),ui->dspin_Iterations->value(),ui->dspin_minRadius->value(),
                                                           ui->dspin_maxRadius->value(),ui->check_optimize->isChecked(),ui->check_negative->isChecked(),
                                                           ui->dspin_distanceWeight->value());
            cloud_view->showInfo("SACSegmentationFromNormals",30,"info");
        }
        break;
    case 1://EuclideanClusterExtraction
        emit EuclideanClusterExtraction(selectedCloud,ui->dspin_tolerance->value(),ui->spin_min_cluster_size->value(),
                                                       ui->spin_max_cluster_size->value());
        cloud_view->showInfo("EuclideanClusterExtraction",30,"info");
        break;
    case 2://RegionGrowing
        emit RegionGrowing(selectedCloud,ui->spin_minclustersize->value(),ui->spin_maxclustersize->value(),ui->check_smoothmode->isChecked(),
                                          ui->check_curvaturetest->isChecked(),ui->check_residualtest->isChecked(),ui->dspin_smooth->value(),
                                          ui->dspin_residual->value(),ui->dspin_curvature->value(),ui->spin_numofnei->value());
        cloud_view->showInfo("RegionGrowing",30,"info");
         break;
    case 3://RegionGrowingRGB
        emit RegionGrowingRGB(selectedCloud,ui->spin_minclustersize->value(),ui->spin_maxclustersize->value(),ui->check_smoothmode->isChecked(),
                                             ui->check_curvaturetest->isChecked(),ui->check_residualtest->isChecked(),ui->dspin_smooth->value(),
                                             ui->dspin_residual->value(),ui->dspin_curvature->value(),ui->spin_numofnei->value(),ui->dspin_pointcolor->value(),
                                             ui->dspin_regioncolor->value(),ui->dspin_distance->value(),ui->spin_nghbr_number->value());
        cloud_view->showInfo("RegionGrowingRGB",30,"info");
        break;
    case 4://MinCutSegmentation
        emit MinCutSegmentation(selectedCloud,selectedCloud->center(),ui->dspin_sigma->value(),ui->dspin_radius->value(),ui->dspin_weight->value(),
                                               ui->spin_neighbour_number->value());
        cloud_view->showInfo("MinCutSegmentation",30,"info");
        break;
    case 5://DonSegmentation
        if(ui->dspin_smallscale->value()>ui->dspin_largescale->value()){
            console->warning(tr("Please set the correct paramters!"));
            return;
        }
        emit DonSegmentation(selectedCloud,selectedCloud->resolution,ui->dspin_smallscale->value(),ui->dspin_largescale->value(),ui->dspin_threshold->value(),
                                            ui->dspin_segradius->value(),ui->spin_mincluster->value(),ui->spin_maxcluster->value());
        cloud_view->showInfo("DonSegmentation",30,"info");
        break;
    case 6://SupervoxelClustering
        emit SupervoxelClustering(selectedCloud,ui->dspin_voxelresolution->value(),ui->dspin_seedresolution->value(),
                                                 ui->dspin_colorimportance->value(),ui->dspin_spatialmportance->value(),ui->dspin_normallmportance->value(),
                                                 ui->check_transform->isChecked());
        cloud_view->showInfo("SupervoxelClustering",30,"info");
        break;
    case 7://MorphologicalFilter
        emit MorphologicalFilter(selectedCloud,ui->spin_maxwinsize->value(),ui->dspin_slope->value(),ui->dspin_maxdistance->value(),
                                                ui->dspin_initialdistance->value(),ui->dspin_cellsize->value(),ui->dspin_base->value(),ui->check_negative->isChecked());
        cloud_view->showInfo("MorphologicalFilter",30,"info");
        break;
    }
    console->showProgressBar();
}

void Segmentations::add()
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    if(selectedCloud->empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(segmented_clouds.empty()) {
        console->warning(tr("Please segment the pointcloud!"));
        return;
    }
    for(size_t i=0;i<segmented_clouds.size();i++) {
        cloud_view->removeCloud("segmented_"+std::to_string(i));
        segmented_clouds[i]->copyInfo(selectedCloud);
        segmented_clouds[i]->prefix("seg"+std::to_string(i)+"-");
        segmented_clouds[i]->update();
        if(i==0)
            cloud_tree->insertCloud(-1,segmented_clouds[i],true);
        else {
            int row=cloud_tree->topLevelItemCount()-1;
            cloud_tree->insertCloud(row,segmented_clouds[i],true);
        }
    }
    console->info(tr("Added successfully!"));
    segmented_clouds.clear();
}

void Segmentations::apply()
{
    Cloud::Ptr selectedCloud=cloud_tree->getSelectedCloud();
    if(selectedCloud->empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(segmented_clouds.empty()) {
        console->warning(tr("Please segment the pointcloud!"));
        return;
    }
    Index index=cloud_tree->getSelectedIndex();
    for (size_t i=0;i<segmented_clouds.size();i++) {
        cloud_view->removeCloud("segmented_"+std::to_string(i));
        segmented_clouds[i]->copyInfo(selectedCloud);
        segmented_clouds[i]->prefix("seg"+std::to_string(i)+"-");
        segmented_clouds[i]->update();
        cloud_tree->insertCloud(index.row,segmented_clouds[i],false);
    }
    cloud_tree->clearCloud(index);
    console->info(tr("Applied successfully!"));
    segmented_clouds.clear();
}

void Segmentations::reset()
{
    cloud_view->removeShape("info");
    if(segmented_clouds.empty()) return;
    for(size_t i=0;i<segmented_clouds.size();i++)
        cloud_view->removeCloud("segmented_"+std::to_string(i));
    segmented_clouds.clear();
}

void Segmentations::result(const std::vector<Cloud::Ptr> &cloud, float time)
{
    console->closeProgressBar();
    console->info(tr("The pointcloud has been segmented to %1 clouds,take time %2 ms").arg(cloud.size()).arg(time));
    if(cloud.size()<=0)return;
    segmented_clouds=cloud;
    for (size_t i=0;i<cloud.size();i++){
        if(cloud[i]->points.size()<=0)continue;
        cloud_view->addCloud(cloud[i],"segmented_"+std::to_string(i));
        cloud_view->setCloudColor(cloud[i],"segmented_"+std::to_string(i),rand()%256,rand()%256,rand()%256);
        cloud_view->setCloudSize("segmented_"+std::to_string(i),3);
    }
}

void Segmentations::closeEvent(QCloseEvent *event)
{
    this->reset();
    cloud_tree->setExtendedSelection(true);
    return QWidget::closeEvent(event);
}

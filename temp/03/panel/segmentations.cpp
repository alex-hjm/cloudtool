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
    qRegisterMetaType<std::vector<Cloud::Ptr>>("std::vector<Cloud::Ptr> &");
    qRegisterMetaType<std::vector<Cloud::Ptr>>("std::vector<Cloud::Ptr>");
    connect(&thread,&QThread::finished,seg,&QObject::deleteLater);
    connect(this,&Segmentations::PlaneSACSegmentation,seg,&Segmentation::PlaneSACSegmentation);
    connect(this,&Segmentations::EuclideanClusterExtraction,seg,&Segmentation::EuclideanClusterExtraction);
    connect(this,&Segmentations::RegionGrowing,seg,&Segmentation::RegionGrowing);
    connect(seg,&Segmentation::result,this,&Segmentations::result);
    thread.start();

    connect(ui->cbox_segmentations,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        if(index==0)
            ui->check_negative->setEnabled(true);
        else {
            ui->check_negative->setChecked(false);
            ui->check_negative->setEnabled(false);
        }
    });
    ui->cbox_segmentations->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
    //RegionGrowing
    connect(ui->check_smoothmode,&QCheckBox::clicked,[=](bool state){
        if(state) ui->dspin_smooth->setEnabled(true);
        else ui->dspin_smooth->setEnabled(false);
    });
    connect(ui->check_curvaturetest,&QCheckBox::clicked,[=](bool state){
        if(state) ui->dspin_curvature->setEnabled(true);
        else ui->dspin_curvature->setEnabled(false);
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
    if(!selectedCloud->has_normals) {
        console->warning(tr("Please estimate the normals of pointcloud first!"));
        return;
    }
    segmented_clouds.clear();
    switch (ui->cbox_segmentations->currentIndex())
    {
    case 0://SACSegmentation
            emit PlaneSACSegmentation(selectedCloud,ui->dspin_distanceWeight->value(),ui->dspin_Threshold->value(),ui->dspin_Iterations->value(),
                                      ui->check_negative->isChecked());
            cloud_view->showInfo("PlaneSACSegmentation",30,"info");
        break;
    case 1://EuclideanClusterExtraction
        emit EuclideanClusterExtraction(selectedCloud,ui->dspin_tolerance->value(),ui->spin_min_cluster_size->value(),ui->spin_max_cluster_size->value());
        cloud_view->showInfo("EuclideanClusterExtraction",30,"info");
        break;
    case 2://RegionGrowing
        emit RegionGrowing(selectedCloud,ui->spin_minclustersize->value(),ui->spin_maxclustersize->value(),ui->check_smoothmode->isChecked(),
                                          ui->check_curvaturetest->isChecked(),ui->dspin_smooth->value(),ui->dspin_curvature->value(),ui->spin_numofnei->value());
        cloud_view->showInfo("RegionGrowing",30,"info");
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

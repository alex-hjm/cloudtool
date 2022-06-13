#include "treesearch.h"
#include "ui_treesearch.h"

TreeSearch::TreeSearch(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TreeSearch),searchCloud(new CloudXYZRGBN),isPicking(false)
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&TreeSearch::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&TreeSearch::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&TreeSearch::reset);
    connect(ui->btn_pickpoint,&QPushButton::clicked,this,&TreeSearch::pick);

    connect(ui->cbox_type,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index) {
        if(index==0){
            ui->rbtn_resolution->hide();
            ui->dspin_resolution->hide();
        } else {
            ui->rbtn_resolution->show();
            ui->dspin_resolution->show();
        }
    });
    ui->rbtn_resolution->hide();
    ui->dspin_resolution->hide();
}

TreeSearch::~TreeSearch()
{
    delete ui;
}

void TreeSearch::init()
{
    cloudTree->setExtendedSelection(false);
    connect(ui->spin_k,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=](int value){
        if(!ui->rbtn_ksearch->isChecked())return;
        Cloud selectedCloud=cloudTree->getSelectedCloud();
        if(selectedCloud.empty()){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        if((ui->spin_point->value()<0)|(ui->spin_point->value()>selectedCloud.size())) {
            console->warning(tr("Please set the correct parameters!"));
            return;
        }
        CloudXYZRGBN::Ptr searchPoint(new CloudXYZRGBN);
        searchPoint->push_back(selectedCloud.cloud->points[ui->spin_point->value()]);
        cloudView->updateCloud(searchPoint,"searchpoint");
        cloudView->setCloudSize("searchpoint",5);
        cloudView->setCloudColor(searchPoint,"searchpoint",255,255,255);
        if(ui->cbox_type->currentIndex()==0) {
            searchCloud=tree.KDTree(selectedCloud.cloud,ui->spin_point->value(),value);
            cloudView->updateCloud(searchCloud,"kdtree");
            cloudView->setCloudSize("kdtree",2);
            cloudView->setCloudColor(searchCloud,"kdtree",255,0,0);
            cloudView->showInfoText("kdtree K-nearest neighbor search",30,"info");
        }
        else if(ui->cbox_type->currentIndex()==1) {
            searchCloud=tree.OCTree(selectedCloud.cloud,ui->spin_point->value(),ui->dspin_resolution->value(),value);
            cloudView->updateCloud(searchCloud,"octree");
            cloudView->setCloudSize("octree",2);
            cloudView->setCloudColor(searchCloud,"octree",0,0,255);
            cloudView->showInfoText("octree K-nearest neighbor search",30,"info");
        }
    });

    connect(ui->dspin_radius,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(!ui->rbtn_rsearch->isChecked())return;
        Cloud selectedCloud=cloudTree->getSelectedCloud();
        if(selectedCloud.empty()){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        if((ui->spin_point->value()<0)|(ui->spin_point->value()>selectedCloud.size())) {
            console->warning(tr("Please set the correct parameters!"));
            return;
        }
        CloudXYZRGBN::Ptr searchPoint(new CloudXYZRGBN);
        searchPoint->push_back(selectedCloud.cloud->points[ui->spin_point->value()]);
        cloudView->updateCloud(searchPoint,"searchpoint");
        cloudView->setCloudSize("searchpoint",5);
        cloudView->setCloudColor(searchPoint,"searchpoint",255,255,255);
        if(ui->cbox_type->currentIndex()==0) {
            searchCloud=tree.KDTree(selectedCloud.cloud,ui->spin_point->value(),float(value));
            cloudView->updateCloud(searchCloud,"kdtree");
            cloudView->setCloudSize("kdtree",2);
            cloudView->setCloudColor(searchCloud,"kdtree",0,255,0);
            cloudView->showInfoText("kdtree R-radius search",30,"info");
        }
        else if(ui->cbox_type->currentIndex()==1) {
            searchCloud=tree.OCTree(selectedCloud.cloud,ui->spin_point->value(),ui->dspin_resolution->value(),float(value));
            cloudView->updateCloud(searchCloud,"octree");
            cloudView->setCloudSize("octree",2);
            cloudView->setCloudColor(searchCloud,"octree",0,255,255);
            cloudView->showInfoText("octree R-radius search",30,"info");
        }
    });

    connect(ui->dspin_resolution,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(!ui->rbtn_resolution->isChecked())return;
        if(!ui->cbox_type->currentIndex()==1)return;
        Cloud selectedCloud=cloudTree->getSelectedCloud();
        if(selectedCloud.empty()){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        if((ui->spin_point->value()<0)|(ui->spin_point->value()>selectedCloud.size())){
            console->warning(tr("Please set the correct parameters!"));
            return;
        }
        CloudXYZRGBN::Ptr searchPoint(new CloudXYZRGBN);
        searchPoint->push_back(selectedCloud.cloud->points[ui->spin_point->value()]);
        cloudView->updateCloud(searchPoint,"searchpoint");
        cloudView->setCloudSize("searchpoint",5);
        cloudView->setCloudColor(searchPoint,"searchpoint",255,255,255);
        searchCloud=tree.OCTree(selectedCloud.cloud,ui->spin_point->value(),float(value));
        cloudView->updateCloud(searchCloud,"octree");
        cloudView->setCloudSize("octree",2);
        cloudView->setCloudColor(searchCloud,"octree",255,255,0);
        cloudView->showInfoText("octree voxel search",30,"info");
    });
}

void TreeSearch::pick()
{
    if(!isPicking){
        Cloud selectedCloud=cloudTree->getSelectedCloud();
        if(selectedCloud.empty()){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        Index index=cloudTree->getSelectedIndex();
        std::vector<Index> allIndexs=cloudTree->getAllIndexs();
        for(int i=0;i<allIndexs.size();i++){
            if(allIndexs[i]!=index)
                cloudTree->setCloudChecked(allIndexs[i],false);
        }
        cloudView->removeShape(selectedCloud.boxid);
        cloudView->removeCloud("octree");
        cloudView->removeCloud("kdtree");
        cloudTree->setEnabled(false);
        isPicking=true;
        cloudView->showInfoText("Point Picking [ON] : Left click",30,"info");
        ui->btn_pickpoint->setText("stop");
        ui->btn_pickpoint->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
        connect(cloudView,&CloudView::mouseLeftPressPos,this,&TreeSearch::leftPressPoint);
    }
    else {
        disconnect(cloudView,&CloudView::mouseLeftPressPos,this,&TreeSearch::leftPressPoint);
        cloudView->removeShape("info");
        cloudTree->setEnabled(true);
        isPicking=false;
        ui->btn_pickpoint->setText("pick");
        ui->btn_pickpoint->setIcon(QIcon(":/icon/resource/icon/start.svg"));
    }

}

void TreeSearch::add()
{
    Cloud selectedCloud=cloudTree->getSelectedCloud();
    if(selectedCloud.empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(searchCloud->empty()){
        console->warning(tr("Please search a pointcloud!"));
        return;
    }
    Index index=cloudTree->getSelectedIndex();
    cloudView->removeShape("info");
    cloudView->removeCloud("octree");
    cloudView->removeCloud("kdtree");
    cloudView->removeCloud("searchpoint");
    Cloud cloud_search(searchCloud,selectedCloud.fileInfo);
    cloud_search.prefix("tree-");
    cloudTree->setCloudChecked(index,false);
    cloudTree->insertCloud(index.row,cloud_search,true);
    cloudView->updateBoundingBox(cloud_search.box,cloud_search.boxid);
}

void TreeSearch::apply()
{
    Cloud selectedCloud=cloudTree->getSelectedCloud();
    if(selectedCloud.empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(searchCloud->empty()){
        console->warning(tr("Please search a pointcloud!"));
        return;
    }
    Index index=cloudTree->getSelectedIndex();
    cloudView->removeShape("info");
    cloudView->removeCloud("octree");
    cloudView->removeCloud("kdtree");
    cloudView->removeCloud("searchpoint");
    *selectedCloud.cloud=*searchCloud;
    cloudTree->updateCloud(index,selectedCloud);
    cloudView->updateBoundingBox(selectedCloud.box,selectedCloud.boxid);
}

void TreeSearch::reset()
{
    if(isPicking) {
        this->pick();
    }
    cloudView->removeShape("info");
    cloudView->removeCloud("octree");
    cloudView->removeCloud("kdtree");
    cloudView->removeCloud("searchpoint");
    searchCloud.reset(new CloudXYZRGBN);
}

void TreeSearch::leftPressPoint(Point2D p)
{
    Cloud selectedCloud=cloudTree->getSelectedCloud();
    int indice=cloudView->singlePick(p);
    ui->spin_point->setValue(indice);
    CloudXYZRGBN::Ptr searchPoint(new CloudXYZRGBN);
    searchPoint->push_back(selectedCloud.cloud->points[indice]);
    cloudView->updateCloud(searchPoint,"searchpoint");
    cloudView->setCloudSize("searchpoint",5);
    cloudView->setCloudColor(searchPoint,"searchpoint",255,255,255);
}

void TreeSearch::closeEvent(QCloseEvent *event)
{
    this->reset();
    cloudTree->setExtendedSelection(true);
    return QWidget::closeEvent(event);
}

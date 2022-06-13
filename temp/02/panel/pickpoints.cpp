#include "pickpoints.h"
#include "ui_pickpoints.h"

PickPoints::PickPoints(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PickPoints),
    isPicking(false),
    pickCloud(new CloudXYZRGBN)
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&PickPoints::add);
    connect(ui->btn_del,&QPushButton::clicked,this,&PickPoints::del);
    connect(ui->btn_start,&QPushButton::clicked,this,&PickPoints::start);
    connect(ui->btn_reset,&QPushButton::clicked,this,&PickPoints::reset);
    connect(ui->btn_close,&QPushButton::clicked,this,&PickPoints::close);
}

PickPoints::~PickPoints()
{
    delete ui;
}

void PickPoints::init()
{
    cloudTree->setExtendedSelection(false);
    ui->check_reverse->setEnabled(false);
    connect(ui->cbox_picktype,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index)
    {
        if(index==0) {
            ui->check_reverse->setChecked(false);
            ui->check_reverse->setEnabled(false);
        } else
            ui->check_reverse->setEnabled(true);
    });
}

void PickPoints::add()
{
    if(isPicking) {
        console->warning(tr("Please stop picking!"));
        return;
    } else {
        if(pickCloud->empty()){
            console->warning(tr("Please pick some points!"));
            return;
        } else {
            cloudView->removeShape("info");
            cloudView->removeShape("info1");
            cloudView->removeShape("pickcloud-Box");
            cloudView->removeCloud("pickcloud");
            index=cloudTree->getSelectedIndex();
            Cloud cloud(selectedCloud.fileInfo,pickCloud);
            cloud.prefix("pick-");
            cloudTree->insertItem(index.row,cloud);
            indices.clear();
            pickCloud.reset(new CloudXYZRGBN);
        }
    }
}

void PickPoints::del()
{
    if(isPicking) {
        console->warning(tr("Please stop picking!"));
        return;
    } else {
        if(pickCloud->empty()){
            console->warning(tr("Please pick some points!"));
            return;
        } else {
            cloudView->removeShape("info");
            cloudView->removeShape("info1");
            cloudView->removeShape("pickcloud-Box");
            cloudView->removeCloud("pickcloud");
            index=cloudTree->getSelectedIndex();
            std::vector<int> indices_reverse;
            tool.GetDifference(selectedCloud.size(),indices,indices_reverse);
            pcl::copyPointCloud(*selectedCloud.cloud,indices_reverse,*selectedCloud.cloud);
            cloudTree->updateItem(index,selectedCloud);
            cloudView->updateCloud(selectedCloud.cloud,selectedCloud.id);
            cloudView->updateBoundingBox(selectedCloud.box,selectedCloud.id+"Box",1,1,255,255,255);
            indices.clear();
            pickCloud.reset(new CloudXYZRGBN);
        }
    }

}

void PickPoints::start()
{
    if(!isPicking){
        selectedCloud=cloudTree->getSelectedCloud();
        if(selectedCloud.empty()){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        allClouds=cloudTree->getAllClouds();
        for(int i=0;i<allClouds.size();i++){
            if(allClouds[i].id!=selectedCloud.id)
                cloudView->removeCloud(allClouds[i].id);
        }
        ui->cbox_picktype->setEnabled(false);
        cloudTree->setEnabled(false);
        isPicking=true;
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
        //remove
        cloudView->removeShape(selectedCloud.id+"Box");
        cloudView->removeShape("pickcloud-Box");
        cloudView->removeCloud("pickcloud");
        //connect
        if(ui->cbox_picktype->currentIndex()==0){
            cloudView->updateInfoText("[point pick]:press shift + left click",30,"info");
            connect(cloudView,&CloudView::pointPicked,this,&PickPoints::pickPoints);
        }
        if(ui->cbox_picktype->currentIndex()==1){
            cloudView->updateInfoText("[area pick]: click X + left click",30,"info");
            connect(cloudView,&CloudView::pointsPicked,this,&PickPoints::pickArea);
        }
        connect(ui->check_reverse,&QCheckBox::clicked,this,&PickPoints::reverseCloud);
        cloudView->updateInfoText("Picking...",50,"info1");
        cloudView->resetCamera();
    } else {
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        ui->cbox_picktype->setEnabled(true);
        cloudTree->setEnabled(true);
        isPicking=false;
        //remove
        cloudView->removeShape("info");
        cloudView->removeShape("info1");
        //disconnect
        disconnect(cloudView,&CloudView::pointPicked,this,&PickPoints::pickPoints);
        disconnect(cloudView,&CloudView::pointsPicked,this,&PickPoints::pickArea);


        cloudView->updateInfoText("Pick over ",50,"info1");

        if(pickCloud->empty()){
            cloudView->updateInfoText("Pick points: 0 ",30,"info");
            return;
        } else {
            cloudView->updateInfoText("Pick points: "+to_string(pickCloud->size()),30,"info");
            if(pickCloud->size()>10){
                BoundingBox box=feature.BoundingBox_AABB(pickCloud);
                cloudView->updateBoundingBox(box,"pickcloud-Box",1,1,255,255,255);
            }
        }
        cloudView->resetCamera();
    }

}

void PickPoints::reset()
{
    cloudView->removeShape("info");
    cloudView->removeShape("info1");
    cloudView->removeShape("pickcloud-Box");
    cloudView->removeCloud("pickcloud");
    disconnect(cloudView,&CloudView::pointPicked,this,&PickPoints::pickPoints);
    disconnect(cloudView,&CloudView::pointsPicked,this,&PickPoints::pickArea);
    disconnect(ui->check_reverse,&QCheckBox::clicked,this,&PickPoints::reverseCloud);
    allClouds=cloudTree->getAllClouds();
    selectedCloud=cloudTree->getSelectedCloud();
    for(int i=0;i<allClouds.size();i++){
        if(!cloudView->contains(allClouds[i].id))
            cloudView->updateCloud(allClouds[i].cloud,allClouds[i].id);
    }
    cloudView->updateBoundingBox(selectedCloud.box,"pickcloud-Box",1,1,255,255,255);
    ui->btn_start->setIcon(QIcon(":/icon/resource/icon/start.svg"));
    indices.clear();
    pickCloud.reset(new CloudXYZRGBN);
    ui->cbox_picktype->setEnabled(true);
    cloudTree->setEnabled(true);
    isPicking=false;
    cloudView->resetCamera();
}

void PickPoints::pickPoints(int index)
{
    pickCloud.reset(new CloudXYZRGBN);
    PointXYZRGBN point=selectedCloud.cloud->points[index];
    pickCloud->push_back(point);
    cloudView->updateCloud(pickCloud,"pickcloud");
    cloudView->setCloudColor(pickCloud,"pickcloud",255,0,0);
    cloudView->setCloudSize("pickcloud",2);
}

void PickPoints::pickArea(std::vector<int> indexs)
{
    pickCloud.reset(new CloudXYZRGBN);
    indices=indexs;
    pcl::copyPointCloud(*selectedCloud.cloud,indices,*pickCloud);
    cloudView->updateCloud(pickCloud,"pickcloud");
    cloudView->setCloudColor(pickCloud,"pickcloud",255,0,0);
    cloudView->setCloudSize("pickcloud",2);
}

void PickPoints::reverseCloud(bool state)
{
    pickCloud.reset(new CloudXYZRGBN);
    if(indices.size()<=0)return;
    if(state) {
        std::vector<int> indices_reverse;
        tool.GetDifference(selectedCloud.size(),indices,indices_reverse);
        pcl::copyPointCloud(*selectedCloud.cloud,indices_reverse,*pickCloud);
    } else
        pcl::copyPointCloud(*selectedCloud.cloud,indices,*pickCloud);
    cloudView->updateCloud(pickCloud,"pickcloud");
    cloudView->setCloudColor(pickCloud,"pickcloud",255,0,0);
    cloudView->setCloudSize("pickcloud",2);
    if(!isPicking) {
        if(pickCloud->size()>10){
            BoundingBox box=feature.BoundingBox_AABB(pickCloud);
            cloudView->updateBoundingBox(box,"pickcloud-Box",1,1,255,255,255);
        }
    }
}

void PickPoints::closeEvent(QCloseEvent *event)
{
    this->reset();
    cloudTree->setExtendedSelection(true);
    return QDialog::closeEvent(event);
}


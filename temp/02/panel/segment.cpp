#include "segment.h"
#include "ui_segment.h"

Segment::Segment(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Segment),segCloud(new CloudXYZRGBN),pickType(0),isPicking(false),pickStart(false)
{
    ui->setupUi(this);
    connect(ui->btn_selectin,&QPushButton::clicked,this,&Segment::selectIn);
    connect(ui->btn_selectout,&QPushButton::clicked,this,&Segment::selectOut);
    connect(ui->btn_add,&QPushButton::clicked,this,&Segment::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Segment::apply);
    connect(ui->btn_start,&QPushButton::clicked,this,&Segment::start);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Segment::reset);
    connect(ui->btn_close,&QPushButton::clicked,this,&Segment::close);

    connect(ui->cbox_type,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index) {
        pickType=index;
        if(index==0) {
            if(isPicking)
                cloudView->showInfoText("Segmentation [ON] (rectangular selection)",30,"info");
            else
                cloudView->showInfoText("Segmentation [OFF] (rectangular selection)",30,"info");
            cloudView->showInfoText("Left/Right click : set opposite corners",50,"info1");
        } else {
            if(isPicking)
                cloudView->showInfoText("Segmentation [ON] (polygonal selection)",30,"info");
            else
                cloudView->showInfoText("Segmentation [OFF] (polygonal selection)",30,"info");
            cloudView->showInfoText("Left click : add contour points / Right click : close",50,"info1");
        }
    });
}

Segment::~Segment()
{
    delete ui;
}

void Segment::init()
{
    cloudTree->setExtendedSelection(false);
    cloudView->showInfoText("Segmentation [OFF] (rectangular selection)",30,"info");
    cloudView->showInfoText("Left/Right click : set opposite corners",50,"info1");

}

void Segment::selectIn()
{
    Cloud selectedCloud=cloudTree->getSelectedCloud();
    if(selectedCloud.empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(points.size()<3) {
        console->warning(tr("Please select an area!"));
        return;
    }
    if(isPicking){
        this->start();
    }
    Index index=cloudTree->getSelectedIndex();
    cloudTree->setCloudChecked(index,false);
    cloudView->setInteractorEnable(true);
    cloudView->removeShape("poly");
    indices=cloudView->areaPick(points,selectedCloud.cloud,false);
    segCloud.reset(new CloudXYZRGBN);
    pcl::copyPointCloud(*selectedCloud.cloud,indices,*segCloud);
    cloudView->updateCloud(segCloud,"seg");
}

void Segment::selectOut()
{
    Cloud selectedCloud=cloudTree->getSelectedCloud();
    if(selectedCloud.empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(points.size()<3) {
        console->warning(tr("Please select an area!"));
        return;
    }
    if(isPicking){
        this->start();
    }
    Index index=cloudTree->getSelectedIndex();
    cloudTree->setCloudChecked(index,false);
    cloudView->setInteractorEnable(true);
    cloudView->removeShape("poly");
    indices=cloudView->areaPick(points,selectedCloud.cloud,true);
    segCloud.reset(new CloudXYZRGBN);
    pcl::copyPointCloud(*selectedCloud.cloud,indices,*segCloud);
    cloudView->updateCloud(segCloud,"seg");
}

void Segment::add()
{
    Cloud selectedCloud=cloudTree->getSelectedCloud();
    if(selectedCloud.empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(segCloud->empty()) {
        console->warning(tr("Please segment a pointcloud!"));
        return;
    }
    Index index=cloudTree->getSelectedIndex();
    cloudView->removeCloud("seg");
    Cloud seg_cloud(segCloud,selectedCloud.fileInfo);
    seg_cloud.prefix("seg-");
    cloudTree->insertCloud(index.row,seg_cloud,true);
    cloudView->updateBoundingBox(seg_cloud.box,seg_cloud.boxid);
}

void Segment::apply()
{
    Cloud selectedCloud=cloudTree->getSelectedCloud();
    if(selectedCloud.empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(segCloud->empty()) {
        console->warning(tr("Please segment a pointcloud!"));
        return;
    }
    Index index=cloudTree->getSelectedIndex();
    cloudView->removeCloud("seg");
    *selectedCloud.cloud=*segCloud;
    cloudTree->updateCloud(index,selectedCloud);
    cloudTree->setCloudChecked(index,true);
    cloudView->updateBoundingBox(selectedCloud.box,selectedCloud.boxid);
}

void Segment::start()
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
        segCloud.reset(new CloudXYZRGBN);
        cloudView->setInteractorEnable(false);
        cloudTree->setEnabled(false);
        isPicking=true;
        points.clear();
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/stop.svg"));
        //connect
        connect(cloudView,&CloudView::mouseLeftPressPos,this,&Segment::leftPressPoint);
        connect(cloudView,&CloudView::mouseLeftReleasePos,this,&Segment::leftReleasePoint);
        connect(cloudView,&CloudView::mouseRightReleasePos,this,&Segment::rightReleasePoint);
        connect(cloudView,&CloudView::mouseMovePos,this,&Segment::movePoint);
        if(pickType==0) {
            cloudView->showInfoText("Segmentation [ON] (rectangular selection)",30,"info");
            cloudView->showInfoText("Left/Right click : set opposite corners",50,"info1");
        } else {
            cloudView->showInfoText("Segmentation [ON] (polygonal selection)",30,"info");
            cloudView->showInfoText("Left click : add contour points / Right click : close",50,"info1");
        }
    } else {
        disconnect(cloudView,&CloudView::mouseLeftPressPos,this,&Segment::leftPressPoint);
        disconnect(cloudView,&CloudView::mouseLeftReleasePos,this,&Segment::leftReleasePoint);
        disconnect(cloudView,&CloudView::mouseRightReleasePos,this,&Segment::rightReleasePoint);
        disconnect(cloudView,&CloudView::mouseMovePos,this,&Segment::movePoint);
        cloudTree->setEnabled(true);
        isPicking=false;
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        if(pickType==0) {
            cloudView->showInfoText("Segmentation [OFF] (rectangular selection)",30,"info");
            cloudView->showInfoText("Left/Right click : set opposite corners",50,"info1");
        } else {
            cloudView->showInfoText("Segmentation [OFF] (polygonal selection)",30,"info");
            cloudView->showInfoText("Left click : add contour points / Right click : close",50,"info1");
        }
    }
}

void Segment::reset()
{
    if(isPicking){
        this->start();
    }
    Index index=cloudTree->getSelectedIndex();
    if(index.row!=-1)
        cloudTree->setCloudChecked(index,true);
    cloudView->setInteractorEnable(true);
    cloudView->removeShape("poly");
    cloudView->removeCloud("seg");
    cloudView->removeCloud("pre");
    segCloud.reset(new CloudXYZRGBN);
    points.clear();
}

void Segment::closeEvent(QCloseEvent *event)
{
    this->reset();
    cloudView->removeShape("info");
    cloudView->removeShape("info1");
    cloudTree->setExtendedSelection(true);
    return QDialog::closeEvent(event);
}

void Segment::leftPressPoint(Point2D pt)
{
    if(!pickStart) {
        points.clear();
        points.push_back(pt);
        pickStart=true;
    }
}

void Segment::leftReleasePoint(Point2D pt)
{
    if(pickStart) {
        if((points[0].x==pt.x)&&(points[0].y==pt.y))
            return;
        else {
            if(pickType==0) {
                //rectangle
                Point2D p1(points[0].x,pt.y);
                Point2D p2(pt.x,points[0].y);
                points.push_back(p1);
                points.push_back(pt);
                points.push_back(p2);
                cloudView->updatePolyLine(points,0,255,0,"poly");
                pickStart=false;
            } else {
                //polygon
                points.push_back(pt);
            }
        }
    }
}

void Segment::rightReleasePoint(Point2D pt)
{
    if(pickStart) {
        if(pickType==0) {
            //rectangle
            Point2D p1(points[0].x,pt.y);
            Point2D p2(pt.x,points[0].y);
            points.push_back(p1);
            points.push_back(pt);
            points.push_back(p2);
            pickStart=false;
        } else {
            //polygon
            if(points.size()==2){
                points.push_back(pt);
                cloudView->updatePolyLine(points,0,255,0,"poly");
                pickStart=false;
            } else if(points.size()>2){
                cloudView->updatePolyLine(points,0,255,0,"poly");
                pickStart=false;
            }
        }
    }

}

void Segment::movePoint(Point2D pt)
{
    if(pickStart) {
        if(pickType==0) {
            //rectangle
            std::vector<Point2D> pre_points=points;
            Point2D p1(pre_points[0].x,pt.y);
            Point2D p2(pt.x,pre_points[0].y);
            pre_points.push_back(p1);
            pre_points.push_back(pt);
            pre_points.push_back(p2);
            cloudView->updatePolyLine(pre_points,0,255,0,"poly");
        } else {
            //polygon
            std::vector<Point2D> pre_points=points;
            pre_points.push_back(pt);
            cloudView->updatePolyLine(pre_points,0,255,0,"poly");
        }
    }
}


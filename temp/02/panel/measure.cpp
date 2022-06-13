#include "measure.h"
#include "ui_measure.h"

Measure::Measure(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Measure),
    isMeasuring(false),pickStart(false)
{
    ui->setupUi(this);
    connect(ui->btn_start,&QPushButton::clicked,this,&Measure::start);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Measure::reset);
    connect(ui->btn_close,&QPushButton::clicked,this,&Measure::close);
}

Measure::~Measure()
{
    delete ui;
}

void Measure::init()
{
    cloudTree->setExtendedSelection(false);
    cloudView->showInfoText("Measure [OFF]  Distance:",30,"info");
    cloudView->showInfoText("Left click : pick  start/end point",50,"info1");
}


void Measure::start()
{
    if(!isMeasuring) {
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
        cloudTree->setEnabled(false);
        isMeasuring=true;
        cloudView->removeShape("dis-arrow");
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/stop.svg"));

        connect(cloudView,&CloudView::mouseLeftPressPos,this,&Measure::leftPressPoint);
        connect(cloudView,&CloudView::mouseLeftReleasePos,this,&Measure::leftReleasePoint);
        connect(cloudView,&CloudView::mouseRightReleasePos,this,&Measure::rightReleasePoint);
        cloudView->showInfoText("Measure [ON]  Distance:",30,"info");
        cloudView->showInfoText("Left click : pick  start/end point",50,"info1");
    } else {
        disconnect(cloudView,&CloudView::mouseLeftPressPos,this,&Measure::leftPressPoint);
        disconnect(cloudView,&CloudView::mouseLeftReleasePos,this,&Measure::leftReleasePoint);
        disconnect(cloudView,&CloudView::mouseRightReleasePos,this,&Measure::rightReleasePoint);
        ui->btn_start->setIcon(QIcon(":/icon/resource/icon/start.svg"));
        cloudTree->setEnabled(true);
        isMeasuring=false;
        cloudView->removeShape("dis-arrow");
        cloudView->removeCloud("startpoint");
        cloudView->removeCloud("endpoint");
        cloudView->showInfoText("Measure [OFF]  Distance:",30,"info");
        cloudView->showInfoText("Left click : pick  start/end point",50,"info1");
    }
}


void Measure::reset()
{
    if(isMeasuring){
        this->start();
    }
    Index index=cloudTree->getSelectedIndex();
    if(index.row!=-1)
        cloudTree->setCloudChecked(index,true);
    ui->txt_distance->clear();
}

void Measure::leftPressPoint(Point2D pt)
{
    if(!pickStart) {
        currentpt=pt;
        CloudXYZRGBN::Ptr point(new CloudXYZRGBN);
        Cloud selectedCloud=cloudTree->getSelectedCloud();
        int index=cloudView->singlePick(pt);
        if(index==-1)return;
        start_point=selectedCloud.cloud->points[index];
        point->push_back(start_point);
        cloudView->updateCloud(point,"startpoint");
        cloudView->setCloudColor(point,"startpoint",255,0,0);
        cloudView->setCloudSize("startpoint",5);
        pickStart=true;
    }
}

void Measure::leftReleasePoint(Point2D pt)
{
    if(pickStart) {
        if((currentpt.x==pt.x)&&(currentpt.y==pt.y))
            return;
        else {
            CloudXYZRGBN::Ptr point(new CloudXYZRGBN);
            Cloud selectedCloud=cloudTree->getSelectedCloud();
            int index=cloudView->singlePick(pt);
            if(index==-1)return;
            end_point=selectedCloud.cloud->points[index];
            point->push_back(end_point);
            cloudView->updateCloud(point,"endpoint");
            cloudView->setCloudColor(point,"endpoint",255,0,0);
            cloudView->setCloudSize("endpoint",5);
            cloudView->updateArrow(end_point,start_point,0,255,0,true,"dis-arrow");
            float x=start_point.x-end_point.x;
            float y=start_point.y-end_point.y;
            float z=start_point.z-end_point.z;
            float distance=sqrt(x*x+y*y+z*z);
            ui->txt_distance->setText(QString::number(distance));
            string dis="Measure [ON]  Distance: "+to_string(distance)+" mm";
            cloudView->showInfoText(dis,30,"info");
            pickStart=false;
        }
    }
}

void Measure::rightReleasePoint(Point2D pt)
{
    if(pickStart) {
        if((currentpt.x==pt.x)&&(currentpt.y==pt.y))
            return;
        else {
            CloudXYZRGBN::Ptr point(new CloudXYZRGBN);
            Cloud selectedCloud=cloudTree->getSelectedCloud();
            int index=cloudView->singlePick(pt);
            if(index==-1)return;
            end_point=selectedCloud.cloud->points[index];
            point->push_back(end_point);
            cloudView->updateCloud(point,"endpoint");
            cloudView->setCloudColor(point,"endpoint",255,0,0);
            cloudView->setCloudSize("endpoint",5);
            cloudView->updateArrow(start_point,end_point,0,255,0,true,"dis-arrow");
            float x=start_point.x-end_point.x;
            float y=start_point.y-end_point.y;
            float z=start_point.z-end_point.z;
            float distance=sqrt(x*x+y*y+z*z);
            ui->txt_distance->setText(QString::number(distance));
            string dis="Measure [ON]  Distance: "+to_string(distance)+" mm";
            cloudView->showInfoText(dis,30,"info");
            pickStart=false;
        }

    }
}

void Measure::closeEvent(QCloseEvent *event)
{
    this->reset();
    cloudView->removeShape("info");
    cloudView->removeShape("info1");
    cloudTree->setExtendedSelection(true);
    return QDialog::closeEvent(event);
}


#include "boundingbox.h"
#include "ui_boundingbox.h"

BoundingBoxs::BoundingBoxs(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::BoundingBox),
    boxType(1)

{
    ui->setupUi(this);
    connect(ui->btn_apply,&QPushButton::clicked,this,&BoundingBoxs::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&BoundingBoxs::reset);
    connect(ui->btn_preview,&QPushButton::clicked,this,&BoundingBoxs::preview);
}

BoundingBoxs::~BoundingBoxs()
{
    delete ui;
}

void BoundingBoxs::init()
{
    connect(ui->cbox_points,&QCheckBox::clicked,[=](bool state) {
        if(state) {
            boxType=0;
            ui->cbox_wireframe->setChecked(false);
            ui->cbox_surface->setChecked(false);
        }
    });
    connect(ui->cbox_wireframe,&QCheckBox::clicked,[=](bool state) {
        if(state) {
            boxType=1;
            ui->cbox_points->setChecked(false);
            ui->cbox_surface->setChecked(false);
        }
    });
    connect(ui->cbox_surface,&QCheckBox::clicked,[=](bool state) {
        if(state) {
            boxType=2;
            ui->cbox_points->setChecked(false);
            ui->cbox_wireframe->setChecked(false);
        }
    });
    connect(ui->dspin_rx,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
        emit eulerAngles(value,ui->dspin_ry->value(),ui->dspin_rz->value());
    });
    connect(ui->dspin_ry,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
        emit eulerAngles(ui->dspin_rx->value(),value,ui->dspin_rz->value());
    });
    connect(ui->dspin_rz,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
        emit eulerAngles(ui->dspin_rx->value(),ui->dspin_ry->value(),value);
    });
}

void BoundingBoxs::preview()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    boundingBoxs.clear();
    this->adjustEnable(false);
    for(size_t i=0;i<selectedClouds.size();i++) {
        BoundingBox box;
        if(ui->rbtn_aabb->isChecked()) {
           box=feature.BoundingBoxAABB(selectedClouds[i].cloud);
           cloudView->updateBoundingBox(box,selectedClouds[i].boxid);
           cloudView->setShapeColor(selectedClouds[i].boxid,255,0,0);
           cloudView->showInfoText("Axis-Aligned Bounding Box",30,"info");
        } else {
           box=feature.BoundingBoxOBB(selectedClouds[i].cloud);
           cloudView->updateBoundingBox(box,selectedClouds[i].boxid);
           cloudView->setShapeColor(selectedClouds[i].boxid,0,255,0);
           cloudView->showInfoText("Oriented Bounding Box",30,"info");
        }
        cloudView->setShapeRepersentation(selectedClouds[i].boxid,boxType);
        if(boxType==0)
            cloudView->setShapeSize(selectedClouds[i].boxid,5);
        else if(boxType==1)
            cloudView->setShapeLineWidth(selectedClouds[i].boxid,3);
        float x,y,z,rx,ry,rz;
        pcl::getTranslationAndEulerAngles(box.affine,x,y,z,rx,ry,rz);
        ui->dspin_rx->setValue(rx/M_PI*180);
        ui->dspin_ry->setValue(ry/M_PI*180);
        ui->dspin_rz->setValue(rz/M_PI*180);
        console->info(tr("BoudingBox has been caculated,take time %1 ms").arg(feature.tocTime));
        boundingBoxs.push_back(box);
    }
    this->adjustEnable(true);
}

void BoundingBoxs::apply()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a correct pointcloud!"));
        return;
    }
    if((boundingBoxs.size()<=0)|(selectedClouds.size()!=boundingBoxs.size())) {
        this->preview();
    }
    cloudView->removeShape("info");
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        selectedClouds[i].box=boundingBoxs[i];
        cloudTree->updateCloud(indexs[i],selectedClouds[i]);
        cloudView->updateBoundingBox(boundingBoxs[i],selectedClouds[i].boxid);
    }
    boundingBoxs.clear();
    this->adjustEnable(false);
}

void BoundingBoxs::reset()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    cloudView->removeShape("info");
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudView->updateBoundingBox(selectedClouds[i].box,selectedClouds[i].boxid);
    }
    boundingBoxs.clear();
    this->adjustEnable(false);
}

void BoundingBoxs::adjustbox(float r,float p,float y)
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    for(size_t i=0;i<selectedClouds.size();i++) {
        Eigen::Affine3f affine=pcl::getTransformation(selectedClouds[i].center()[0],selectedClouds[i].center()[1],
                selectedClouds[i].center()[2],r/180*M_PI,p/180*M_PI,y/180*M_PI);
        BoundingBox box=feature.BoundingBoxAdjust(selectedClouds[i].cloud,affine.inverse());
        cloudView->updateBoundingBox(box,selectedClouds[i].boxid);
        cloudView->setShapeColor(selectedClouds[i].boxid,0,0,255);
        cloudView->setShapeRepersentation(selectedClouds[i].boxid,boxType);
        if(boxType==0)
            cloudView->setShapeSize(selectedClouds[i].boxid,5);
        else if(boxType==1)
            cloudView->setShapeLineWidth(selectedClouds[i].boxid,3);
        boundingBoxs[i]=box;
    }
}

void BoundingBoxs::adjustEnable(bool state)
{
    if(state) {
        connect(this,&BoundingBoxs::eulerAngles,this,&BoundingBoxs::adjustbox);
        ui->dspin_rx->setEnabled(true);
        ui->dspin_ry->setEnabled(true);
        ui->dspin_rz->setEnabled(true);
    }
    else{
        disconnect(this,&BoundingBoxs::eulerAngles,this,&BoundingBoxs::adjustbox);
        ui->dspin_rx->setEnabled(false);
        ui->dspin_ry->setEnabled(false);
        ui->dspin_rz->setEnabled(false);
    }
}


void BoundingBoxs::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QWidget::closeEvent(event);
}

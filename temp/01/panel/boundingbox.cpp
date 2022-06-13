#include "boundingbox.h"
#include "ui_boundingbox.h"

BoundingBox::BoundingBox(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::BoundingBox),box_type(1)
{
    ui->setupUi(this);
    connect(ui->btn_apply,&QPushButton::clicked,this,&BoundingBox::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&BoundingBox::reset);
    connect(ui->btn_preview,&QPushButton::clicked,this,&BoundingBox::preview);
    connect(ui->check_points,&QCheckBox::clicked,[=](bool state) {
        if(state) {
            box_type=0;
            ui->check_wireframe->setChecked(false);
            ui->check_surface->setChecked(false);
        }
    });
    connect(ui->check_wireframe,&QCheckBox::clicked,[=](bool state) {
        if(state) {
            box_type=1;
            ui->check_points->setChecked(false);
            ui->check_surface->setChecked(false);
        }
    });
    connect(ui->check_surface,&QCheckBox::clicked,[=](bool state) {
        if(state) {
            box_type=2;
            ui->check_points->setChecked(false);
            ui->check_wireframe->setChecked(false);
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

BoundingBox::~BoundingBox()
{
    delete ui;
}

void BoundingBox::init(Console* &co,CloudView* &cv,CloudTree* &ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
}

void BoundingBox::preview()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    boundingBoxs.clear();
    this->adjustEnable(false);
    for(auto &i:selectedClouds) {
        Box box;
        if(ui->rbtn_aabb->isChecked()) {
            box=Features::boundingBoxAABB(i);
            cloud_view->updateCube(box,i->box_id);
            cloud_view->setShapeColor(i->box_id,255,0,0);
            cloud_view->showInfo("Axis-Aligned Bounding Box",30,"info");
        } else {
            box=Features::boundingBoxOBB(i);
            cloud_view->updateCube(box,i->box_id);
            cloud_view->setShapeColor(i->box_id,0,255,0);
            cloud_view->showInfo("Oriented Bounding Box",30,"info");
        }
        cloud_view->setShapeRepersentation(i->box_id,box_type);
        if(box_type==0)
            cloud_view->setShapeSize(i->box_id,5);
        else if(box_type==1)
            cloud_view->setShapeLineWidth(i->box_id,3);
        float x,y,z,rx,ry,rz;
        pcl::getTranslationAndEulerAngles(box.affine,x,y,z,rx,ry,rz);
        ui->dspin_rx->setValue(rx/M_PI*180);
        ui->dspin_ry->setValue(ry/M_PI*180);
        ui->dspin_rz->setValue(rz/M_PI*180);
        boundingBoxs.push_back(box);
    }
    this->adjustEnable(true);
}

void BoundingBox::apply()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a correct pointcloud!"));
        return;
    }
    if(boundingBoxs.size()<=0||selectedClouds.size()!=boundingBoxs.size()) {
        this->preview();
    }
    cloud_view->removeShape("info");
    for(size_t i=0;i<selectedClouds.size();i++) {
        selectedClouds[i]->box=boundingBoxs[i];
        cloud_view->updateCube(boundingBoxs[i],selectedClouds[i]->box_id);
    }
    console->info(tr("Applied successfully!"));
    boundingBoxs.clear();
    this->adjustEnable(false);
}

void BoundingBox::reset()
{
    cloud_view->removeShape("info");
    boundingBoxs.clear();
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    for(auto &i:selectedClouds) {
        cloud_view->updateCube(i->box,i->box_id);
    }
    this->adjustEnable(false);
}

void BoundingBox::adjustEnable(bool state)
{
    if(state) {
        connect(this,&BoundingBox::eulerAngles,this,&BoundingBox::adjustBox);
        ui->dspin_rx->setEnabled(true);
        ui->dspin_ry->setEnabled(true);
        ui->dspin_rz->setEnabled(true);
    }
    else{
        disconnect(this,&BoundingBox::eulerAngles,this,&BoundingBox::adjustBox);
        ui->dspin_rx->setEnabled(false);
        ui->dspin_ry->setEnabled(false);
        ui->dspin_rz->setEnabled(false);
    }
}

void BoundingBox::adjustBox(float r,float p,float y)
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    for(size_t i=0;i<selectedClouds.size();i++) {
        Eigen::Affine3f affine=pcl::getTransformation(selectedClouds[i]->center()[0],selectedClouds[i]->center()[1],
                selectedClouds[i]->center()[2],r/180*M_PI,p/180*M_PI,y/180*M_PI);
        Box box=Features::boundingBoxAdjust(selectedClouds[i],affine.inverse());
        cloud_view->updateCube(box,selectedClouds[i]->box_id);
        cloud_view->setShapeColor(selectedClouds[i]->box_id,0,0,255);
        cloud_view->setShapeRepersentation(selectedClouds[i]->box_id,box_type);
        if(box_type==0)
            cloud_view->setShapeSize(selectedClouds[i]->box_id,5);
        else if(box_type==1)
            cloud_view->setShapeLineWidth(selectedClouds[i]->box_id,3);
        boundingBoxs[i]=box;
    }
}

void BoundingBox::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QDockWidget::closeEvent(event);
}

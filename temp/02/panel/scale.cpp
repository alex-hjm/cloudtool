#include "scale.h"
#include "ui_scale.h"

Scale::Scale(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Scale)
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&Scale::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Scale::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Scale::reset);
    connect(ui->btn_close,&QPushButton::clicked,this,&Scale::close);
    connect(ui->check_samevalue,&QCheckBox::stateChanged,[=](int state){
        if(state){
            ui->dspin_y->setEnabled(false);
            ui->dspin_z->setEnabled(false);
        }
        else {
            ui->dspin_y->setEnabled(true);
            ui->dspin_z->setEnabled(true);
        }
    });
    connect(ui->dspin_x,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(ui->check_samevalue->isChecked()){
            ui->dspin_y->setValue(value);
            ui->dspin_z->setValue(value);
            emit scale(value,value,value);
        }
        emit scale(value,ui->dspin_y->value(),ui->dspin_z->value());
    });
    connect(ui->dspin_y,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(ui->check_samevalue->isChecked())
            return ;
        emit scale(ui->dspin_x->value(),value,ui->dspin_z->value());
    });

    connect(ui->dspin_z,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value)
    {
        if(ui->check_samevalue->isChecked())
            return ;
        emit scale(ui->dspin_x->value(),ui->dspin_y->value(),value);
    });
    ui->check_samevalue->setChecked(true);
}

Scale::~Scale()
{
    delete ui;
}

void Scale::init()
{

    connect(this,&Scale::scale,this,&Scale::scaleCloud);
    connect(cloudTree,&CloudTree::removedId,this,&Scale::removeCloud);
}

void Scale::add()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((scaledClouds.size()<=0)|(selectedClouds.size()!=scaledClouds.size()))
        this->scaleCloud(ui->dspin_x->value(),ui->dspin_y->value(),ui->dspin_z->value());
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudView->removeCloud(selectedClouds[i].id+"-scaled");
        Cloud cloud(scaledClouds[i],selectedClouds[i].fileInfo);
        cloud.prefix("scaled-");
        cloudTree->insertCloud(indexs[i].row,cloud,true);
        cloudView->updateBoundingBox(cloud.box,cloud.boxid);


    }
    scaledClouds.clear();
}

void Scale::apply()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((scaledClouds.size()<=0)|(selectedClouds.size()!=scaledClouds.size()))
        this->scaleCloud(ui->dspin_x->value(),ui->dspin_y->value(),ui->dspin_z->value());
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudTree->setCloudChecked(indexs[i],true);
        cloudView->removeCloud(selectedClouds[i].id+"-scaled");
        *selectedClouds[i].cloud=*scaledClouds[i];
        cloudTree->updateCloud(indexs[i],selectedClouds[i]);
        cloudView->updateBoundingBox(selectedClouds[i].box,selectedClouds[i].boxid);
    }
}

void Scale::reset()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) return;
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(!cloudView->contains(selectedClouds[i].id))
            cloudTree->setCloudChecked(indexs[i],true);
        cloudView->removeCloud(selectedClouds[i].id+"-scaled");
    }
    scaledClouds.clear();
}

void Scale::scaleCloud(double x, double y, double z)
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    scaledClouds.clear();
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(cloudView->contains(selectedClouds[i].id)){
           cloudTree->setCloudChecked(indexs[i],false);
        }
        CloudXYZRGBN::Ptr scaledCloud(new CloudXYZRGBN);
        if(ui->cbox_type->currentIndex()==0)
            scaledCloud=com.ScaleCloud(selectedClouds[i].cloud,selectedClouds[i].center(),x,y,z,false);
        else if(ui->cbox_type->currentIndex()==1)
            scaledCloud=com.ScaleCloud(selectedClouds[i].cloud,selectedClouds[i].center(),x,y,z,true);
        if(ui->check_keepentity->isChecked())
            cloudView->resetCamera();
        cloudView->updateCloud(scaledCloud,selectedClouds[i].id+"-scaled");
        scaledClouds.push_back(scaledCloud);
    }
}

void Scale::removeCloud(const string &id)
{
    cloudView->removeCloud(id+"-scaled");
}

void Scale::closeEvent(QCloseEvent *event)
{
   this->reset();
   return QDialog::closeEvent(event);
}

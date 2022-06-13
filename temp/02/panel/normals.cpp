#include "normals.h"
#include "ui_normals.h"

Normals::Normals(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Normals),
    viewpoint(Eigen::Vector3f::Identity())
{
    ui->setupUi(this);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Normals::preview);
    connect(ui->btn_add,&QPushButton::clicked,this,&Normals::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Normals::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Normals::reset);

    connect(ui->check_max,&QCheckBox::clicked,[=](bool state) {
        if(state) {
            ui->check_center->setChecked(false);
            ui->check_origin->setChecked(false);
        }
    });
    connect(ui->check_center,&QCheckBox::clicked,[=](bool state) {
        if(state) {
            ui->check_max->setChecked(false);
            ui->check_origin->setChecked(false);
        }

    });
    connect(ui->check_origin,&QCheckBox::clicked,[=](bool state) {
        if(state) {
            ui->check_max->setChecked(false);
            ui->check_center->setChecked(false);
        }
    });

    connect(ui->spin_level,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=](int value) {
        if(ui->check_refresh->isChecked())
            this->showNormals();

    });
    connect(ui->dspin_scale,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value) {
        if(ui->check_refresh->isChecked())
            this->showNormals();
    });
    connect(ui->rbtn_k,&QRadioButton::clicked,[=](bool checked){
        if(checked) {
            ui->spin_k->setEnabled(true);
            ui->dspin_r->setValue(0);
            ui->dspin_r->setEnabled(false);
        }
    });
    connect(ui->rbtn_r,&QRadioButton::clicked,[=](bool checked){
        if(checked) {
            ui->dspin_r->setEnabled(true);
            ui->spin_k->setValue(0);
            ui->spin_k->setEnabled(false);
        }
    });
}

Normals::~Normals()
{
    delete ui;
}

void Normals::init()
{
    connect(ui->check_reverse,&QCheckBox::stateChanged,[=]()
    {
        std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
        if(selectedClouds.size()<=0) {
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        if((cloudsWithNormals.size()<=0)|(cloudsWithNormals.size()!=selectedClouds.size())){
            console->warning(tr("Please estimation the normals or select the correct pointcloud!"));
            return;
        }
        for(size_t i=0;i<selectedClouds.size();i++) {
            for(size_t j=0;j<cloudsWithNormals[i]->points.size();j++){
                cloudsWithNormals[i]->points[j].normal_x=-cloudsWithNormals[i]->points[j].normal_x;
                cloudsWithNormals[i]->points[j].normal_y=-cloudsWithNormals[i]->points[j].normal_y;
                cloudsWithNormals[i]->points[j].normal_z=-cloudsWithNormals[i]->points[j].normal_z;
            }
            cloudView->updateNormols(cloudsWithNormals[i],ui->spin_level->value(),ui->dspin_scale->value(),selectedClouds[i].normalsid);
        }
    });
}

void Normals::preview()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(ui->spin_k->value()==0&&ui->dspin_r->value()==0) {
        console->warning(tr("Please set the correct parameters!"));
        return;
    }
    cloudsWithNormals.clear();
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(ui->check_origin->isChecked())
            viewpoint<<selectedClouds[i].cloud->sensor_origin_.coeff(0),selectedClouds[i].cloud->sensor_origin_.coeff(1),
                    selectedClouds[i].cloud->sensor_origin_.coeff(2);
        else if(ui->check_center->isChecked())
            viewpoint=selectedClouds[i].center();
        else if(ui->check_max)
            viewpoint<<std::numeric_limits<float>::max (),std::numeric_limits<float>::max (),std::numeric_limits<float>::max ();
        else
            viewpoint<<selectedClouds[i].cloud->sensor_origin_.coeff(0),selectedClouds[i].cloud->sensor_origin_.coeff(1),
                    selectedClouds[i].cloud->sensor_origin_.coeff(2);
        CloudXYZRGBN::Ptr normals(new CloudXYZRGBN);
        normals=feature.NormalEstimation(selectedClouds[i].cloud,ui->spin_k->value(),ui->dspin_r->value(),viewpoint);
        if(ui->check_reverse->isChecked()){
            for(size_t j=0;j<normals->points.size();j++){
                normals->points[j].normal_x=-normals->points[j].normal_x;
                normals->points[j].normal_y=-normals->points[j].normal_y;
                normals->points[j].normal_z=-normals->points[j].normal_z;
            }
        }
        if(ui->rbtn_k->isChecked())
            cloudView->showInfoText("K-nearest neighbor search estimation",30,"info");
        else
            cloudView->showInfoText("R-radius search estimation",30,"info");
        cloudView->updateNormols(normals,ui->spin_level->value(),ui->dspin_scale->value(),selectedClouds[i].normalsid);
        cloudsWithNormals.push_back(normals);
        console->info(tr("The pointcloud ")+selectedClouds[i].id.c_str()+tr(" estimate normals done,take time %1 ms.").arg(feature.tocTime));
    }
}

void Normals::add()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((cloudsWithNormals.size()<=0)|(selectedClouds.size()!=cloudsWithNormals.size())) {
        console->warning(tr("Please estimation the normals or select the correct pointcloud!"));
        return;
    }
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    cloudView->removeShape("info");
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudView->removeCloud(selectedClouds[i].normalsid);
        Cloud cloud(cloudsWithNormals[i],selectedClouds[i].fileInfo);
        cloud.prefix("normals-");
        cloudTree->insertCloud(indexs[i].row,cloud,true);
        cloudView->updateBoundingBox(cloud.box,cloud.boxid);
    }
    cloudsWithNormals.clear();
}

void Normals::apply()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((cloudsWithNormals.size()<=0)|(selectedClouds.size()!=cloudsWithNormals.size())) {
        this->preview();
    }
    cloudView->removeShape("info");
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudView->removeCloud(selectedClouds[i].normalsid);
        *selectedClouds[i].cloud=*cloudsWithNormals[i];
        cloudTree->updateCloud(indexs[i],selectedClouds[i]);
        cloudView->updateBoundingBox(selectedClouds[i].box,selectedClouds[i].boxid);
    }
    cloudsWithNormals.clear();
}

void Normals::reset()
{
    cloudView->removeShape("info");
    cloudsWithNormals.clear();
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudView->removeCloud(selectedClouds[i].normalsid);
    }
}

void Normals::showNormals()
{
    std::vector<Cloud>  selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((cloudsWithNormals.size()<=0)|(selectedClouds.size()!=cloudsWithNormals.size())) {
        console->warning(tr("Please estimation the normals or select the correct pointcloud!"));
        return;
    }
    for(size_t i=0;i<selectedClouds.size();i++)
        cloudView->updateNormols(cloudsWithNormals[i],ui->spin_level->value(),ui->dspin_scale->value(),selectedClouds[i].normalsid);
}

void Normals::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QWidget::closeEvent(event);
}

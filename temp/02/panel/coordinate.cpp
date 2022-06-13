#include "coordinate.h"
#include "ui_coordinate.h"

Coordinate::Coordinate(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Coordinate)
{
    ui->setupUi(this);
    connect(ui->btn_add,&QPushButton::clicked,this,&Coordinate::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Coordinate::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Coordinate::reset);
    connect(ui->btn_close,&QPushButton::clicked,this,&Coordinate::close);
}

Coordinate::~Coordinate()
{
    delete ui;
}

void Coordinate::init()
{
    connect(ui->dspin_coord,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
        if(ui->cbox_type->currentIndex()==0){
            if((selectedClouds.size()<=0)|(affines.size()<=0)){
                cloudView->updateCoord(value,"Coord");
                return;
            }
            for(size_t i=0;i<affines.size();i++) {
                cloudView->updateCoord(value,affines[i],selectedClouds[i].id+"-coord");
            }
        } else {
            for(size_t i=0;i<selectedClouds.size();i++) {
                cloudView->setCloudSize(selectedClouds[i].id,value);
            }
        }
    });

    cloudView->updateCoord(ui->dspin_coord->value(),"Coord");
    connect(cloudTree,&CloudTree::removedId,this,&Coordinate::removeCloud);
}

void Coordinate::add()
{
    if(ui->cbox_type->currentIndex()!=0)return;
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    affines.clear();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudView->updateCoord(ui->dspin_coord->value(),selectedClouds[i].box.affine,selectedClouds[i].id+"-coord");
        affines.push_back(selectedClouds[i].box.affine);
    }
}

void Coordinate::apply()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    if(ui->cbox_type->currentIndex()==0) {
        if(affines.size()<=0)this->add();
        for(size_t i=0;i<selectedClouds.size();i++) {
            cloudView->removeCoord(selectedClouds[i].id+"-coord");
            pcl::transformPointCloud(*selectedClouds[i].cloud,*selectedClouds[i].cloud,affines[i].inverse());
            cloudTree->updateCloud(indexs[i],selectedClouds[i]);
            cloudView->updateBoundingBox(selectedClouds[i].box,selectedClouds[i].boxid);
        }
        affines.clear();
    } else {
        for(size_t i=0;i<selectedClouds.size();i++) {
            cloudView->setCloudSize(selectedClouds[i].id,ui->dspin_coord->value());
            selectedClouds[i].pointSize=ui->dspin_coord->value();
            cloudTree->updateCloud(indexs[i],selectedClouds[i]);
            cloudView->updateBoundingBox(selectedClouds[i].box,selectedClouds[i].boxid);
         }
    }

}

void Coordinate::reset()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()>0){
        for(size_t i=0;i<selectedClouds.size();i++) {
            if(ui->cbox_type->currentIndex()==0)
                cloudView->removeCoord(selectedClouds[i].id+"-coord");
            else
                cloudView->setCloudSize(selectedClouds[i].id,1);
        }
    }
    affines.clear();
    cloudView->updateCoord(ui->dspin_coord->value(),"Coord");
}

void Coordinate::removeCloud(const string &id)
{
    cloudView->removeCloud(id+"-coord");
}

void Coordinate::closeEvent(QCloseEvent * event)
{
    this->reset();
    cloudView->removeCoord("Coord");
    return QDialog::closeEvent(event);
}



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


void Scale::init(Console* &co,CloudView* &cv,CloudTree* &ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
    connect(this,&Scale::scale,this,&Scale::scaleCloud);
    connect(cloud_tree,&CloudTree::removedId,this,&Scale::removeCloud);
}

void Scale::add()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(scaledClouds.empty()||selectedClouds.size()!=scaledClouds.size())
        scaleCloud(ui->dspin_x->value(),ui->dspin_y->value(),ui->dspin_z->value());
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<scaledClouds.size();i++) {
        cloud_view->removeCloud(selectedClouds[i]->id+"-scaled");
        scaledClouds[i]->prefix("scaled-");
        scaledClouds[i]->update();
        cloud_tree->insertCloud(indexs[i].row,scaledClouds[i],true);
        cloud_view->updateCube(scaledClouds[i]->box,scaledClouds[i]->box_id);
    }
    console->info(tr("Added successfully!"));
    scaledClouds.clear();
}

void Scale::apply()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(scaledClouds.size()<=0||selectedClouds.size()!=scaledClouds.size())
        scaleCloud(ui->dspin_x->value(),ui->dspin_y->value(),ui->dspin_z->value());
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloud_view->removeCloud(selectedClouds[i]->id+"-scaled");
        selectedClouds[i]->swap(*scaledClouds[i]);
        selectedClouds[i]->update();
        cloud_tree->setCloudChecked(indexs[i],true);
        cloud_view->updateCube(selectedClouds[i]->box,selectedClouds[i]->box_id);
    }
    console->info(tr("Applied successfully!"));
    scaledClouds.clear();
}

void Scale::reset()
{
    scaledClouds.clear();
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(!cloud_view->contains(selectedClouds[i]->id))
            cloud_tree->setCloudChecked(indexs[i],true);
        cloud_view->removeCloud(selectedClouds[i]->id+"-scaled");
    }
}

void Scale::scaleCloud(double x, double y, double z)
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    scaledClouds.clear();
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(cloud_view->contains(selectedClouds[i]->id)){
           cloud_tree->setCloudChecked(indexs[i],false);
        }
        Cloud::Ptr scaledCloud;
        if(ui->cbox_type->currentIndex()==0)
            scaledCloud=Common::ScaleCloud(selectedClouds[i],x,y,z,false);
        else if(ui->cbox_type->currentIndex()==1)
            scaledCloud=Common::ScaleCloud(selectedClouds[i],x,y,z,true);
        if(ui->check_keepentity->isChecked())
            cloud_view->resetCamera();
        cloud_view->addCloud(scaledCloud,scaledCloud->id+"-scaled");
        scaledClouds.push_back(scaledCloud);
    }
}

void Scale::removeCloud(const std::string &id)
{
    cloud_view->removeCloud(id+"-scaled");
}

void Scale::closeEvent(QCloseEvent *event)
{
   this->reset();
   return QDialog::closeEvent(event);
}

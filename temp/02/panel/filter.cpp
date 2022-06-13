#include "filter.h"
#include "ui_filter.h"

Filter::Filter(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Filter)
{
    ui->setupUi(this);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Filter::preview);
    connect(ui->btn_add,&QPushButton::clicked,this,&Filter::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Filter::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Filter::reset);

    //PassThrough
    connect(ui->dspin_min_limit,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
        if(ui->check_refresh->isChecked())
            this->preview();
    });
    connect(ui->dspin_max_limit,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
        if(ui->check_refresh->isChecked())
            this->preview();
    });
    //VoxelGrid
    connect(ui->checkbox_same_value,&QCheckBox::stateChanged,[=](int state){
        if(state){
            ui->dspin_leafy->setEnabled(false);
            ui->dspin_leafz->setEnabled(false);
        }
        else {
            ui->dspin_leafy->setEnabled(true);
            ui->dspin_leafz->setEnabled(true);
        }
    });
    connect(ui->dspin_leafx,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double value){
        if(ui->checkbox_same_value->isChecked()&&ui->check_refresh->isChecked()){
            ui->dspin_leafy->setValue(value);
            ui->dspin_leafz->setValue(value);
            this->preview();
        }
    });
    connect(ui->dspin_leafy,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
        if(!ui->checkbox_same_value->isChecked()&&ui->check_refresh->isChecked())
            this->preview();
    });
    connect(ui->dspin_leafz,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
        if(!ui->checkbox_same_value->isChecked()&&ui->check_refresh->isChecked())
            this->preview();
    });
    connect(ui->checkbox_approximate,&QCheckBox::stateChanged,[=](int state){
        if(state)
            ui->checkbox_reverse->setEnabled(false);
        else
            ui->checkbox_reverse->setEnabled(true);
    });

    //StatisticalOutlierRemoval
    connect(ui->spin_meanK,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=]{
        if(ui->check_refresh->isChecked())
            this->preview();
    });
    connect(ui->dspin_StddevMulThresh,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
        if(ui->check_refresh->isChecked())
            this->preview();
    });
    //RadiusOutlierRemoval
    connect(ui->dspin_Radius,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
        if(ui->check_refresh->isChecked())
            this->preview();
    });
    connect(ui->spin_MinNeiborsInRadius,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=]{
        if(ui->check_refresh->isChecked())
            this->preview();
    });
}

Filter::~Filter()
{
    delete ui;
}

void Filter::init()
{
    //PassThrough
    connect(ui->combox_fieid_name,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int Index){
        std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
        if(selectedClouds.size()<=0) return;
        float min_x=std::numeric_limits<float>::max (),max_x=-std::numeric_limits<float>::max (),
                min_y=std::numeric_limits<float>::max (),max_y=-std::numeric_limits<float>::max (),
                min_z=std::numeric_limits<float>::max (),max_z=-std::numeric_limits<float>::max ();
        for(size_t i=0;i<selectedClouds.size();i++) {
            if(selectedClouds[i].min.x<min_x) min_x=selectedClouds[i].min.x;
            if(selectedClouds[i].min.y<min_y) min_y=selectedClouds[i].min.y;
            if(selectedClouds[i].min.z<min_z) min_z=selectedClouds[i].min.z;
            if(selectedClouds[i].max.x>max_x) max_x=selectedClouds[i].max.x;
            if(selectedClouds[i].max.y>max_y) max_y=selectedClouds[i].max.y;
            if(selectedClouds[i].max.z>max_z) max_z=selectedClouds[i].max.z;
        }
        switch (Index){
        case 0://x
            ui->dspin_min_limit->setValue(min_x);
            ui->dspin_max_limit->setValue(max_x);
            break;
        case 1://y
            ui->dspin_min_limit->setValue(min_y);
            ui->dspin_max_limit->setValue(max_y);
            break;
        case 2://z
            ui->dspin_min_limit->setValue(min_z);
            ui->dspin_max_limit->setValue(max_z);
            break;
        case 3://rgb
            ui->dspin_min_limit->setValue(0);
            ui->dspin_max_limit->setValue(1);
            break;
        case 4://curvature
            ui->dspin_min_limit->setValue(0);
            ui->dspin_max_limit->setValue(1);
            break;
        }
    });
    ui->filtersWidget->setCurrentIndex(0);
    ui->combox_filters->setCurrentIndex(0);
    ui->check_refresh->setChecked(true);
    connect(cloudTree,&CloudTree::removedId,this,&Filter::removeCloud);
}

void Filter::preview()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    int currentIndex=ui->combox_filters->currentIndex();
    filteredClouds.clear();
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(cloudView->contains(selectedClouds[i].id))
            cloudTree->setCloudChecked(indexs[i],false);
        CloudXYZRGBN::Ptr filteredCloud(new CloudXYZRGBN);
        switch (currentIndex){
        case 0://PassThrough
            filteredCloud=filters.PassThrough(selectedClouds[i].cloud,ui->combox_fieid_name->currentText().toStdString(),
                                              ui->dspin_min_limit->value(),ui->dspin_max_limit->value(),ui->checkbox_reverse->isChecked());
            cloudView->showInfoText("PassThrough",30,"info");
            break;
        case 1://VoxelGrid
            if(!ui->checkbox_approximate->isChecked())
                filteredCloud=filters.VoxelGrid(selectedClouds[i].cloud,ui->dspin_leafx->value(),ui->dspin_leafy->value(),
                                                ui->dspin_leafz->value(),ui->checkbox_reverse->isChecked());
            else
                filteredCloud=filters.ApproximateVoxelGrid(selectedClouds[i].cloud,ui->dspin_leafx->value(),ui->dspin_leafy->value(),
                                                           ui->dspin_leafz->value());
            cloudView->showInfoText("VoxelGrid",30,"info");
            break;
        case 2://StatisticalOutlierRemoval
            filteredCloud=filters.StatisticalOutlierRemoval(selectedClouds[i].cloud,ui->spin_meanK->value(),
                                                            ui->dspin_StddevMulThresh->value(),ui->checkbox_reverse->isChecked());
            cloudView->showInfoText("StatisticalOutlierRemoval",30,"info");
            break;
        case 3://RadiusOutlierRemoval
            filteredCloud=filters.RadiusOutlierRemoval(selectedClouds[i].cloud,ui->dspin_Radius->value(),
                                                       ui->spin_MinNeiborsInRadius->value(),ui->checkbox_reverse->isChecked());
            cloudView->showInfoText("RadiusOutlierRemoval",30,"info");
            break;
        }
        cloudView->updateCloud(filteredCloud,selectedClouds[i].id+"-filtered");
        filteredClouds.push_back(filteredCloud);
        if(!ui->check_refresh->isChecked())
            console->info(tr("The pointcloud ")+selectedClouds[i].id.c_str()+tr(" has been filtered,take time %1 ms").arg(filters.tocTime));
    }
}

void Filter::add()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((filteredClouds.size()<=0)|(selectedClouds.size()!=filteredClouds.size())){
        console->warning(tr("Please filter the pointcloud!"));
        return;
    }
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    cloudView->removeShape("info");
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudView->removeCloud(selectedClouds[i].id+"-filtered");
        Cloud cloud(filteredClouds[i],selectedClouds[i].fileInfo);
        cloud.prefix("filtered-");
        cloudTree->insertCloud(indexs[i].row,cloud,true);
        cloudView->updateBoundingBox(cloud.box,cloud.boxid);
    }
    filteredClouds.clear();
}

void Filter::apply()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((filteredClouds.size()<=0)|(selectedClouds.size()!=filteredClouds.size())){
        this->preview();
    }
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    cloudView->removeShape("info");
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudTree->setCloudChecked(indexs[i],true);
        cloudView->removeCloud(selectedClouds[i].id+"-filtered");
        *selectedClouds[i].cloud=*filteredClouds[i];
        cloudTree->updateCloud(indexs[i],selectedClouds[i]);
        cloudView->updateBoundingBox(selectedClouds[i].box,selectedClouds[i].boxid);
    }
    filteredClouds.clear();
}

void Filter::reset()
{
    cloudView->removeShape("info");
    filteredClouds.clear();
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudTree->setCloudChecked(indexs[i],true);
        cloudView->removeCloud(selectedClouds[i].id+"-filtered");
    }
}

void Filter::removeCloud(const string &id)
{
    cloudView->removeCloud(id+"-filtered");
}


void Filter::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QWidget::closeEvent(event);
}


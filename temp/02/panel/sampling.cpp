#include "sampling.h"
#include "ui_sampling.h"

Sampling::Sampling(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Sampling)
{
    ui->setupUi(this);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Sampling::preview);
    connect(ui->btn_add,&QPushButton::clicked,this,&Sampling::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Sampling::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Sampling::reset);
    connect(ui->btn_close,&QPushButton::clicked,this,&Sampling::close);
    connect(ui->cbox_type,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        switch (index) {
        case 0:
            ui->dspin_ratio->hide();ui->spin_seed->hide();this->resize(180,83);
            ui->dspin_value->setPrefix("radius: ");
            ui->dspin_value->setDecimals(3);
            ui->dspin_value->setSingleStep(0.001);
            break;
        case 1:
            ui->dspin_ratio->hide();ui->spin_seed->hide();this->resize(180,83);
            ui->dspin_value->setPrefix("leaf size: ");
            ui->dspin_value->setDecimals(3);
            ui->dspin_value->setSingleStep(0.001);
            break;
        case 2:
            ui->dspin_ratio->hide();ui->spin_seed->show();this->resize(180,110);
            ui->dspin_value->setPrefix("sample: ");
            ui->dspin_value->setDecimals(0);
            ui->dspin_value->setSingleStep(1);
            ui->spin_seed->setPrefix("seed: ");
            break;
        case 3:
            ui->dspin_ratio->show();ui->spin_seed->show();this->resize(180,110);
            ui->dspin_value->setPrefix("sample: ");
            ui->dspin_value->setDecimals(0);
            ui->dspin_value->setSingleStep(1);
            ui->spin_seed->setPrefix("seed: ");
            ui->dspin_ratio->setPrefix("ratio: ");
            ui->dspin_ratio->setDecimals(3);
            ui->dspin_ratio->setSingleStep(0.001);
            break;
        case 4:
            ui->dspin_ratio->show();ui->spin_seed->show();this->resize(180,110);
            ui->dspin_value->setPrefix("sample: ");
            ui->dspin_value->setDecimals(0);
            ui->dspin_value->setSingleStep(1);
            ui->spin_seed->setPrefix("seed: ");
            ui->dspin_ratio->setPrefix("bins: ");
            ui->dspin_ratio->setDecimals(0);
            ui->dspin_ratio->setSingleStep(1);
            break;
        }
    });
    ui->cbox_type->setCurrentIndex(0);
    ui->dspin_ratio->hide();ui->spin_seed->hide();this->resize(180,83);
    ui->dspin_value->setPrefix("radius: ");
}

Sampling::~Sampling()
{
    delete ui;
}

void Sampling::init(){

}

void Sampling::preview()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    int currentIndex=ui->cbox_type->currentIndex();
    samplingClouds.clear();
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        CloudXYZRGBN::Ptr samplingCloud(new CloudXYZRGBN);
        switch (currentIndex){
        case 0://UniformSampling
            samplingCloud=sam.UniformSampling(selectedClouds[i].cloud,ui->dspin_value->value());
            cloudView->showInfoText("UniformSampling",30,"info");
            break;
        case 1://DownSampling
            samplingCloud=sam.DownSampling(selectedClouds[i].cloud,ui->dspin_value->value());
            cloudView->showInfoText("DownSampling",30,"info");
            break;
        case 2://RandomSampling
            samplingCloud=sam.RandomSampling(selectedClouds[i].cloud,int(ui->dspin_value->value()),ui->spin_seed->value());
            cloudView->showInfoText("RandomSampling",30,"info");
            break;
        case 3://SamplingSurfaceNormal
            if(!selectedClouds[i].hasNormals) {
                console->warning(tr("Please estimation the normals for the pointcloud first!"));
                cloudTree->setCloudChecked(indexs[i],true);
                return;
            }
            samplingCloud=sam.SamplingSurfaceNormal(selectedClouds[i].cloud,int(ui->dspin_value->value()),ui->spin_seed->value(),
                                                    float(ui->dspin_ratio->value()));
            cloudView->showInfoText("SamplingSurfaceNormal",30,"info");
            break;
        case 4://NormalSpaceSampling
            if(!selectedClouds[i].hasNormals) {
                console->warning(tr("Please estimation the normals for the pointcloud first!"));
                cloudTree->setCloudChecked(indexs[i],true);
                return;
            }
            samplingCloud=sam.NormalSpaceSampling(selectedClouds[i].cloud,int(ui->dspin_value->value()),ui->spin_seed->value(),
                                                  ui->dspin_ratio->value());
            cloudView->showInfoText("NormalSpaceSampling",30,"info");
            break;
        }
        if(ui->check_askeypoints->isChecked()) {
            if(!cloudView->contains(selectedClouds[i].id))
                cloudTree->setCloudChecked(indexs[i],true);
            cloudView->updateCloud(samplingCloud,selectedClouds[i].keyid);
            cloudView->setCloudColor(samplingCloud,selectedClouds[i].keyid,0,0,255);
            cloudView->setCloudSize(selectedClouds[i].keyid,5);
        } else {
            if(cloudView->contains(selectedClouds[i].id))
                cloudTree->setCloudChecked(indexs[i],false);
            cloudView->removeCloud(selectedClouds[i].keyid);
            cloudView->updateCloud(samplingCloud,selectedClouds[i].id+"-sampled");
        }
        console->info(tr("The pointcloud ")+selectedClouds[i].id.c_str()+tr(" has been sampled,take time %1 ms").arg(sam.tocTime));
        samplingClouds.push_back(samplingCloud);
    }
}

void Sampling::add()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(samplingClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((samplingClouds.size()<=0)|(selectedClouds.size()!=samplingClouds.size())){
        console->warning(tr("Please sampling the pointcloud!"));
        return;
    }
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    cloudView->removeShape("info");
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudView->removeCloud(selectedClouds[i].id+"-sampled");
        cloudView->removeCloud(selectedClouds[i].keyid);
        Cloud cloud(samplingClouds[i],selectedClouds[i].fileInfo);
        cloud.prefix("sampled-");
        cloudTree->insertCloud(indexs[i].row,cloud,true);
        cloudView->updateBoundingBox(cloud.box,cloud.boxid);
    }
    samplingClouds.clear();
}

void Sampling::apply()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((samplingClouds.size()<=0)|(selectedClouds.size()!=samplingClouds.size())){
        this->preview();
    }
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    cloudView->removeShape("info");
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudTree->setCloudChecked(indexs[i],true);
        cloudView->removeCloud(selectedClouds[i].id+"-sampled");
        cloudView->removeCloud(selectedClouds[i].keyid);
        if(ui->check_askeypoints->isChecked()) {
            selectedClouds[i].keypoints=samplingClouds[i];
            console->info(tr("Applied keypoints for the pointcloud ")+selectedClouds[i].id.c_str()+tr(" (point size: %1)").arg(samplingClouds[i]->size()));
        } else {
            *selectedClouds[i].cloud=*samplingClouds[i];
            console->info(tr("Applied sampling for the pointcloud ")+selectedClouds[i].id.c_str()+tr(" (point size: %1)").arg(samplingClouds[i]->size()));
        }
        cloudTree->updateCloud(indexs[i],selectedClouds[i]);
        cloudView->updateBoundingBox(selectedClouds[i].box,selectedClouds[i].boxid);
    }
    samplingClouds.clear();
}

void Sampling::reset()
{
    cloudView->removeShape("info");
    samplingClouds.clear();
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloudTree->setCloudChecked(indexs[i],true);
        cloudView->removeCloud(selectedClouds[i].id+"-sampled");
        cloudView->removeCloud(selectedClouds[i].keyid);
    }
}

void Sampling::removeCloud(const string &id)
{
    cloudView->removeCloud(id+"-sampled");
}

void Sampling::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QWidget::closeEvent(event);
}

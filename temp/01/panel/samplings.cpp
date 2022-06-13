#include "samplings.h"
#include "ui_samplings.h"

Samplings::Samplings(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Samplings)
{
    ui->setupUi(this);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Samplings::preview);
    connect(ui->btn_add,&QPushButton::clicked,this,&Samplings::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Samplings::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Samplings::reset);
    connect(ui->btn_close,&QPushButton::clicked,this,&Samplings::close);
    Sampling *sampling=new Sampling;
    sampling->moveToThread(&thread);
    connect(&thread,&QThread::finished,sampling,&QObject::deleteLater);
    connect(this,&Samplings::uniformSampling,sampling,&Sampling::uniformSampling);
    connect(this,&Samplings::downSampling,sampling,&Sampling::downSampling);
    connect(this,&Samplings::randomSampling,sampling,&Sampling::randomSampling);
    connect(this,&Samplings::samplingSurfaceNormal,sampling,&Sampling::samplingSurfaceNormal);
    connect(this,&Samplings::normalSpaceSampling,sampling,&Sampling::normalSpaceSampling);
    connect(sampling,&Sampling::result,this,&Samplings::result);
    thread.start();
    connect(ui->cbox_type,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        switch (index) {
        case 0:
            ui->dspin_ratio->hide();ui->spin_seed->hide();this->setFixedSize(180,83);
            ui->dspin_value->setPrefix("radius: ");
            ui->dspin_value->setDecimals(3);
            ui->dspin_value->setSingleStep(0.001);
            break;
        case 1:
            ui->dspin_ratio->hide();ui->spin_seed->hide();this->setFixedSize(180,83);
            ui->dspin_value->setPrefix("leaf size: ");
            ui->dspin_value->setDecimals(3);
            ui->dspin_value->setSingleStep(0.001);
            break;
        case 2:
            ui->dspin_ratio->hide();ui->spin_seed->show();this->setFixedSize(180,110);
            ui->dspin_value->setPrefix("sample: ");
            ui->dspin_value->setDecimals(0);
            ui->dspin_value->setSingleStep(1);
            ui->spin_seed->setPrefix("seed: ");
            break;
        case 3:
            ui->dspin_ratio->show();ui->spin_seed->show();this->setFixedSize(180,110);
            ui->dspin_value->setPrefix("sample: ");
            ui->dspin_value->setDecimals(0);
            ui->dspin_value->setSingleStep(1);
            ui->spin_seed->setPrefix("seed: ");
            ui->dspin_ratio->setPrefix("ratio: ");
            ui->dspin_ratio->setDecimals(3);
            ui->dspin_ratio->setSingleStep(0.001);
            break;
        case 4:
            ui->dspin_ratio->show();ui->spin_seed->show();this->setFixedSize(180,110);
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
    ui->dspin_value->setPrefix("radius: ");
    ui->cbox_type->setCurrentIndex(0);
    ui->dspin_ratio->hide();ui->spin_seed->hide();this->setFixedSize(180,83);
}

Samplings::~Samplings()
{
    thread.quit();
    thread.wait();
    delete ui;
}

void Samplings::init(Console* &co,CloudView* &cv,CloudTree* &ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
}

void Samplings::preview()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(cloud_view->contains(selectedClouds[i]->id))
            cloud_tree->setCloudChecked(indexs[i],false);
        switch (ui->cbox_type->currentIndex()){
        case 0://UniformSampling
            emit uniformSampling(selectedClouds[i],ui->dspin_value->value());
            cloud_view->showInfo("UniformSampling",30,"info");
            break;
        case 1://DownSampling
            emit downSampling(selectedClouds[i],ui->dspin_value->value());
            cloud_view->showInfo("DownSampling",30,"info");
            break;
        case 2://RandomSampling
            emit randomSampling(selectedClouds[i],int(ui->dspin_value->value()),ui->spin_seed->value());
            cloud_view->showInfo("RandomSampling",30,"info");
            break;
        case 3://SamplingSurfaceNormal
            if(!selectedClouds[i]->has_normals) {
                console->warning(tr("Please estimation the normals for the pointcloud first!"));
                cloud_tree->setCloudChecked(indexs[i],true);
                return;
            }
            emit samplingSurfaceNormal(selectedClouds[i],int(ui->dspin_value->value()),ui->spin_seed->value(),
                                       float(ui->dspin_ratio->value()));
            cloud_view->showInfo("SamplingSurfaceNormal",30,"info");
            break;
        case 4://NormalSpaceSampling
            if(!selectedClouds[i]->has_normals) {
                console->warning(tr("Please estimation the normals for the pointcloud first!"));
                cloud_tree->setCloudChecked(indexs[i],true);
                return;
            }
            emit normalSpaceSampling(selectedClouds[i],int(ui->dspin_value->value()),ui->spin_seed->value(),
                                     ui->dspin_ratio->value());
            cloud_view->showInfo("NormalSpaceSampling",30,"info");
            break;
        }
        console->showProgressBar();
    }
}

void Samplings::add()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(sampling_clouds.empty()||selectedClouds.size()!=sampling_clouds.size()){
        console->warning(tr("Please sampling the pointcloud!"));
        return;
    }
    cloud_view->removeShape("info");
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloud_view->removeCloud(selectedClouds[i]->id+"-sampled");
        sampling_clouds[i]->prefix("sampled-");
        sampling_clouds[i]->update();
        cloud_tree->insertCloud(indexs[i].row,sampling_clouds[i],true);
        cloud_view->updateCube(sampling_clouds[i]->box,sampling_clouds[i]->box_id);
    }
    console->info(tr("Added successfully!"));
    sampling_clouds.clear();
}

void Samplings::apply()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(sampling_clouds.empty()||selectedClouds.size()!=sampling_clouds.size()){
        this->preview();
        return;
    }
    cloud_view->removeShape("info");
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloud_tree->setCloudChecked(indexs[i],true);
        cloud_view->removeCloud(selectedClouds[i]->id+"-sampled");
        selectedClouds[i]->swap(*sampling_clouds[i]);
        selectedClouds[i]->update();
        cloud_view->updateCloud(selectedClouds[i],selectedClouds[i]->id);
        cloud_view->updateCube(selectedClouds[i]->box,selectedClouds[i]->box_id);
    }
    console->info(tr("Applied successfully!"));
    sampling_clouds.clear();
}

void Samplings::reset()
{
    cloud_view->removeShape("info");
    sampling_clouds.clear();
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(!cloud_view->contains(selectedClouds[i]->id))
        cloud_tree->setCloudChecked(indexs[i],true);
        cloud_view->removeCloud(selectedClouds[i]->id+"-sampled");
    }
}

void Samplings::result(const Cloud::Ptr &cloud, float time)
{
    console->closeProgressBar();
    cloud_view->addCloud(cloud,cloud->id+"-sampled");
    sampling_clouds.push_back(cloud);
    console->info(tr("The pointcloud ")+cloud->id.c_str()+tr(" has been sampled,take time %1 ms").arg(time));

}

void Samplings::removeCloud(const std::string &id)
{
    cloud_view->removeCloud(id+"-sampled");
}

void Samplings::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QDialog::closeEvent(event);
}

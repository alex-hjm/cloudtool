#include "normals.h"
#include "ui_normals.h"

Normals::Normals(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::Normals)
{
    ui->setupUi(this);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Normals::preview);
    connect(ui->btn_add,&QPushButton::clicked,this,&Normals::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Normals::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Normals::reset);
    Features *feature=new Features;
    feature->moveToThread(&thread);
    qRegisterMetaType<Eigen::Vector3f>("Eigen::Vector3f &");
    qRegisterMetaType<Eigen::Vector3f>("Eigen::Vector3f");
    connect(&thread,&QThread::finished,feature,&QObject::deleteLater);
    connect(this,&Normals::normalEstimation,feature,&Features::normalEstimation);
    connect(feature,&Features::result,this,&Normals::result);
    thread.start();
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

    connect(ui->spin_level,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=](int) {
        if(ui->check_refresh->isChecked())
            this->updateNormals();
    });
    connect(ui->dspin_scale,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=](double) {
        if(ui->check_refresh->isChecked())
            this->updateNormals();
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
    thread.quit();
    thread.wait();
    delete ui;
}

void Normals::init(Console *&co, CloudView *&cv, CloudTree *&ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
    connect(ui->check_reverse,&QCheckBox::stateChanged,[=]()
    {
        std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
        if(selectedClouds.empty()){
            console->warning(tr("Please select a pointcloud!"));
            return;
        }
        if(clouds_with_normals.empty()||clouds_with_normals.size()!=selectedClouds.size()){
            console->warning(tr("Please estimation the normals or select the correct pointcloud!"));
            return;
        }
        for(auto&i:clouds_with_normals) {
            for(auto&j:i->points){
                j.normal_x=-j.normal_x;j.normal_y=-j.normal_y;j.normal_z=-j.normal_z;
            }
            cloud_view->updateCloudNormals(i,ui->spin_level->value(),ui->dspin_scale->value(),i->normals_id);
        }
    });
}

void Normals::preview()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(ui->spin_k->value()==0&&ui->dspin_r->value()==0) {
        console->warning(tr("Please set the correct parameters!"));
        return;
    }
    clouds_with_normals.clear();
    for(auto&i:selectedClouds) {
        Eigen::Vector3f viewpoint;
        if(ui->check_origin->isChecked())
            viewpoint<<i->sensor_origin_.coeff(0),i->sensor_origin_.coeff(1), i->sensor_origin_.coeff(2);
        else if(ui->check_center->isChecked())
            viewpoint=i->center();
        else if(ui->check_max)
            viewpoint<<std::numeric_limits<float>::max (),std::numeric_limits<float>::max (),std::numeric_limits<float>::max ();
        emit normalEstimation(i,ui->spin_k->value(),ui->dspin_r->value(),viewpoint);
        if(ui->rbtn_k->isChecked())
            cloud_view->showInfo("K-nearest neighbor search estimation",30,"info");
        else
            cloud_view->showInfo("R-radius search estimation",30,"info");
        console->showProgressBar();
    }
}

void Normals::add()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(clouds_with_normals.empty()||selectedClouds.size()!=clouds_with_normals.size()) {
        console->warning(tr("Please estimation the normals or select the correct pointcloud!"));
        return;
    }
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    cloud_view->removeShape("info");
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloud_view->removeCloud(selectedClouds[i]->normals_id);
        clouds_with_normals[i]->prefix("normals-");
        clouds_with_normals[i]->update();
        cloud_tree->insertCloud(indexs[i].row,clouds_with_normals[i],true);
        cloud_view->updateCube(clouds_with_normals[i]->box,clouds_with_normals[i]->box_id);
    }
    console->info(tr("Added successfully!"));
    clouds_with_normals.clear();
}

void Normals::apply()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()){
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(clouds_with_normals.size()<=0||selectedClouds.size()!=clouds_with_normals.size()) {
        this->preview();
        return;
    }
    cloud_view->removeShape("info");
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloud_view->removeCloud(selectedClouds[i]->normals_id);
        selectedClouds[i]->swap(*clouds_with_normals[i]);
        selectedClouds[i]->update();
        cloud_view->updateCube(selectedClouds[i]->box,selectedClouds[i]->box_id);
    }
    console->info(tr("Applied successfully!"));
    clouds_with_normals.clear();
}

void Normals::reset()
{
    cloud_view->removeShape("info");
    clouds_with_normals.clear();
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    for(auto&i:selectedClouds)
        cloud_view->removeCloud(i->normals_id);
}

void Normals::result(const Cloud::Ptr &cloud, float time)
{
    console->closeProgressBar();
    if(ui->check_reverse->isChecked()){
        for(auto&j:cloud->points){
            j.normal_x=-j.normal_x;j.normal_y=-j.normal_y;j.normal_z=-j.normal_z;
        }
    }
    cloud_view->addCloudNormals(cloud,ui->spin_level->value(),ui->dspin_scale->value(),cloud->normals_id);
    clouds_with_normals.push_back(cloud);
    console->info(tr("The pointcloud ")+cloud->id.c_str()+tr(" estimate normals done,take time %1 ms.").arg(time));
}

void Normals::updateNormals()
{
    std::vector<Cloud::Ptr>  selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(clouds_with_normals.size()<=0||selectedClouds.size()!=clouds_with_normals.size()) {
        console->warning(tr("Please estimation the normals or select the correct pointcloud!"));
        return;
    }
    for(auto&i:clouds_with_normals){
        cloud_view->updateCloudNormals(i,ui->spin_level->value(),ui->dspin_scale->value(),i->normals_id);
    }

}

void Normals::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QDockWidget::closeEvent(event);
}

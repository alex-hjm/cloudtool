#include "filters.h"
#include "ui_filters.h"

Filters::Filters(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::Filters)
{
    ui->setupUi(this);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Filters::preview);
    connect(ui->btn_add,&QPushButton::clicked,this,&Filters::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Filters::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Filters::reset);
    Filter *filter=new Filter;
    filter->moveToThread(&thread);
    qRegisterMetaType<std::string>("std::string &");
    qRegisterMetaType<std::string>("std::string");
    connect(&thread,&QThread::finished,filter,&QObject::deleteLater);
    connect(this,&Filters::passThrough,filter,&Filter::passThrough);
    connect(this,&Filters::voxelGrid,filter,&Filter::voxelGrid);
    connect(this,&Filters::approximateVoxelGrid,filter,&Filter::approximateVoxelGrid);
    connect(this,&Filters::statisticalOutlierRemoval,filter,&Filter::statisticalOutlierRemoval);
    connect(this,&Filters::radiusOutlierRemoval,filter,&Filter::radiusOutlierRemoval);
    connect(this,&Filters::movingLeastSquares,filter,&Filter::MovingLeastSquares);
    connect(filter,&Filter::result,this,&Filters::result);
    thread.start();
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
    connect(ui->check_same_value,&QCheckBox::stateChanged,[=](int state){
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
        if(ui->check_same_value->isChecked()){
            ui->dspin_leafy->setValue(value);
            ui->dspin_leafz->setValue(value);
        }
        if(ui->check_refresh->isChecked())
            this->preview();
    });
    connect(ui->dspin_leafy,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
        if(!ui->check_same_value->isChecked()&&ui->check_refresh->isChecked())
            this->preview();
    });
    connect(ui->dspin_leafz,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
        if(!ui->check_same_value->isChecked()&&ui->check_refresh->isChecked())
            this->preview();
    });
    connect(ui->check_approximate,&QCheckBox::stateChanged,[=](int state){
        if(state)
            ui->check_reverse->setEnabled(false);
        else
            ui->check_reverse->setEnabled(true);
    });
    //StatisticalOutlierRemoval
    connect(ui->spin_meank,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=]{
        if(ui->check_refresh->isChecked())
            this->preview();
    });
    connect(ui->dspin_stddevmulthresh,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
        if(ui->check_refresh->isChecked())
            this->preview();
    });
    //RadiusOutlierRemoval
    connect(ui->dspin_radius,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
        if(ui->check_refresh->isChecked())
            this->preview();
    });
    connect(ui->spin_minneiborsinradius,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=]{
        if(ui->check_refresh->isChecked())
            this->preview();
    });
    //RadiusOutlierRemoval
    connect(ui->dspin_radius_2,static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),[=]{
        if(ui->check_refresh->isChecked())
            this->preview();
    });
    connect(ui->spin_polynomialOrder,static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),[=]{
        if(ui->check_refresh->isChecked())
            this->preview();
    });
}

Filters::~Filters()
{
    thread.quit();
    thread.wait();
    delete ui;
}

void Filters::init(Console* &co,CloudView* &cv,CloudTree* &ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
    //PassThrough
    connect(ui->cbox_fieid_name,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
            this,&Filters::getRangeFromCloud);
    this->getRangeFromCloud(ui->cbox_fieid_name->currentIndex());
    ui->filtersWidget->setCurrentIndex(0);
    ui->cbox_filters->setCurrentIndex(0);
    ui->check_refresh->setChecked(true);
    connect(cloud_tree,&CloudTree::removedId,this,&Filters::removeCloud);
}

void Filters::preview()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    filtered_clouds.clear();
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(cloud_view->contains(selectedClouds[i]->id))
            cloud_tree->setCloudChecked(indexs[i],false);
        switch (ui->cbox_filters->currentIndex()){
        case 0://PassThrough
            emit passThrough(selectedClouds[i],ui->cbox_fieid_name->currentText().toStdString(),
                             ui->dspin_min_limit->value(),ui->dspin_max_limit->value(),ui->check_reverse->isChecked());
            cloud_view->showInfo("PassThrough",30,"info");
            break;
        case 1://VoxelGrid
            if(!ui->check_approximate->isChecked())
                emit voxelGrid(selectedClouds[i],ui->dspin_leafx->value(),ui->dspin_leafy->value(),
                               ui->dspin_leafz->value(),ui->check_reverse->isChecked());
            else
                emit approximateVoxelGrid(selectedClouds[i],ui->dspin_leafx->value(),ui->dspin_leafy->value(),
                                          ui->dspin_leafz->value());
            cloud_view->showInfo("VoxelGrid",30,"info");
            break;
        case 2://StatisticalOutlierRemoval
            emit statisticalOutlierRemoval(selectedClouds[i],ui->spin_meank->value(),
                                           ui->dspin_stddevmulthresh->value(),ui->check_reverse->isChecked());
            cloud_view->showInfo("StatisticalOutlierRemoval",30,"info");
            break;
        case 3://RadiusOutlierRemoval
            emit radiusOutlierRemoval(selectedClouds[i],ui->dspin_radius->value(),
                                      ui->spin_minneiborsinradius->value(),ui->check_reverse->isChecked());
            cloud_view->showInfo("RadiusOutlierRemoval",30,"info");
            break;
        case 4://MovingLeastSquares
            emit movingLeastSquares(selectedClouds[i],ui->check_computeNormals->isChecked(),
                                      ui->spin_polynomialOrder->value(),ui->dspin_radius_2->value());
            cloud_view->showInfo("RadiusOutlierRemoval",30,"info");
            break;
        }
        if(!ui->check_refresh->isChecked())
            console->showProgressBar();
    }
}

void Filters::add()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(filtered_clouds.empty()||selectedClouds.size()!=filtered_clouds.size()){
        console->warning(tr("Please filter the pointcloud first!"));
        return;
    }
    cloud_view->removeShape("info");
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloud_view->removeCloud(selectedClouds[i]->id+"-filtered");
        filtered_clouds[i]->prefix("filtered-");
        filtered_clouds[i]->update();
        cloud_tree->insertCloud(indexs[i].row,filtered_clouds[i],true);
        cloud_view->updateCube(filtered_clouds[i]->box,filtered_clouds[i]->box_id);
    }
    console->info(tr("Added successfully!"));
    filtered_clouds.clear();
}

void Filters::apply()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(filtered_clouds.empty()||selectedClouds.size()!=filtered_clouds.size()){
        this->preview();
        return;
    }
    cloud_view->removeShape("info");
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        cloud_tree->setCloudChecked(indexs[i],true);
        cloud_view->removeCloud(selectedClouds[i]->id+"-filtered");
        selectedClouds[i]->swap(*filtered_clouds[i]);
        selectedClouds[i]->update();
        cloud_view->updateCloud(selectedClouds[i],selectedClouds[i]->id);
        cloud_view->updateCube(selectedClouds[i]->box,selectedClouds[i]->box_id);
    }
    console->info(tr("Applied successfully!"));
    filtered_clouds.clear();
}

void Filters::reset()
{
    cloud_view->removeShape("info");
    filtered_clouds.clear();
    std::vector<Cloud::Ptr>selectedClouds=cloud_tree->getSelectedClouds();
    std::vector<Index> indexs=cloud_tree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(!cloud_view->contains(selectedClouds[i]->id))
            cloud_tree->setCloudChecked(indexs[i],true);
        cloud_view->removeCloud(selectedClouds[i]->id+"-filtered");
    }
}

void Filters::result(const Cloud::Ptr &cloud, float time)
{
    cloud_view->addCloud(cloud,cloud->id+"-filtered");
    filtered_clouds.push_back(cloud);
    if(!ui->check_refresh->isChecked()) {
        console->closeProgressBar();
        console->info(tr("The pointcloud ")+cloud->id.c_str()+tr(" has been filtered,take time %1 ms").arg(time));
    }
}

void Filters::getRangeFromCloud(int index)
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) return;
    float min=std::numeric_limits<float>::max (),max=-std::numeric_limits<float>::max ();
    switch (index){
    case 0://x
        for(auto &i:selectedClouds) {
            if(i->min.x<min) min=i->min.x;if(i->max.x>max) max=i->max.x;
        }
        ui->dspin_min_limit->setRange(min,max);
        ui->dspin_max_limit->setRange(min,max);
        ui->dspin_min_limit->setValue(min);
        ui->dspin_max_limit->setValue(max);
        break;
    case 1://y
        for(auto &i:selectedClouds) {
            if(i->min.y<min) min=i->min.y;if(i->max.y>max) max=i->max.y;
        }
        ui->dspin_min_limit->setRange(min,max);
        ui->dspin_max_limit->setRange(min,max);
        ui->dspin_min_limit->setValue(min);
        ui->dspin_max_limit->setValue(max);
        break;
    case 2://z
        for(auto &i:selectedClouds) {
            if(i->min.z<min) min=i->min.z;if(i->max.z>max) max=i->max.z;
        }
        ui->dspin_min_limit->setRange(min,max);
        ui->dspin_max_limit->setRange(min,max);
        ui->dspin_min_limit->setValue(min);
        ui->dspin_max_limit->setValue(max);
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
}

void Filters::removeCloud(const std::string &id)
{
    cloud_view->removeCloud(id+"-filtered");
}

void Filters::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QDockWidget::closeEvent(event);
}


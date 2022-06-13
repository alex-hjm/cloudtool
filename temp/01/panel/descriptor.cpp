#include "descriptor.h"
#include "ui_descriptor.h"

Descriptor::Descriptor(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::Descriptor)
{
    ui->setupUi(this);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Descriptor::preview);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Descriptor::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Descriptor::reset);
    connect(ui->rbtn_r,&QRadioButton::clicked,[=](bool checked){
        if(checked) {
            ui->dspin_r->setEnabled(true);
            ui->spin_k->setEnabled(false);
            ui->spin_k->setValue(0);
        }
    });
    connect(ui->rbtn_k,&QRadioButton::clicked,[=](bool checked){
        if(checked) {
            ui->spin_k->setEnabled(true);
            ui->dspin_r->setEnabled(false);
            ui->dspin_r->setValue(0);
        }
    });
    ui->cbox_feature_type->setCurrentIndex(0);
}

Descriptor::~Descriptor()
{
    delete ui;
}

void Descriptor::init(Console* &co,CloudView* &cv,CloudTree* &ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
}

void Descriptor::preview()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    int feature_type=ui->cbox_feature_type->currentIndex();
    if(feature_type==1) fpfhs.clear();
    if(feature_type==3) shots.clear();
    plotter.reset(new pcl::visualization::PCLPlotter("Histogram"));
    std::vector<double> array_x(500), array_y(500);
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(!selectedClouds[i]->has_normals) {
            console->warning(tr("Please Estimation the normals of pointcloud first!"));
            continue;
        }
        switch (feature_type){
        case 0://PFHEstimation
            pfh.reset(new PFHFeature);
            plotter->setTitle("Point Feature Histogram");
            pfh=Features::PFHEstimation(selectedClouds[i],ui->spin_k->value(),ui->dspin_r->value());
            if(!pfh->empty())
                plotter->addFeatureHistogram(*pfh,500);
            break;
        case 1://FPFHEstimation
            fpfh.reset(new FPFHFeature);
            plotter->setTitle("Fast Point Feature Histogram");
            fpfh=Features::FPFHEstimation(selectedClouds[i],ui->spin_k->value(),ui->dspin_r->value());
            if(!fpfh->empty()) {
                fpfhs.push_back(fpfh);
                plotter->addFeatureHistogram(*fpfh,500);
            }
            break;
        case 2://VFHEstimation
            vfh.reset(new VFHFeature);
            plotter->setTitle("Viewpoint Feature Histogram");
            vfh=Features::VFHEstimation(selectedClouds[i],ui->spin_k->value(),ui->dspin_r->value());
            if(!fpfh->empty())
                plotter->addFeatureHistogram(*vfh,500);
            break;
        case 3://SHOTEstimation
            shot.reset(new SHOTFeature);
            shot=Features::SHOTEstimation(selectedClouds[i],ui->spin_k->value(),ui->dspin_r->value());
            if(!shot->empty()) {
                shots.push_back(shot);
                for (int i = 0; i < 500; ++i){
                    array_x[i] = i;
                    array_y[i] = (*shot)[0].descriptor[i];
                }
                plotter->addPlotData(array_x, array_y, "cloud", vtkChart::LINE);
            }
            break;
        }
    }
    plotter->setShowLegend (false);
    plotter->setWindowPosition((QApplication::desktop()->width()-plotter->getWindowSize()[0])/2,
            (QApplication::desktop()->height()-plotter->getWindowSize()[1])/2);
    plotter->plot();
}

void Descriptor::apply()
{
    std::vector<Cloud::Ptr> selectedClouds=cloud_tree->getSelectedClouds();
    if(selectedClouds.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(ui->cbox_feature_type->currentIndex()==1) {//fpfh
        if(fpfhs.empty()||selectedClouds.size()!=fpfhs.size())
            this->preview();
        for(size_t i=0;i<selectedClouds.size();i++){
            selectedClouds[i]->setFeature(fpfhs[i]);
            console->info(tr("Applied the fpfh feature to pointcloud of ")+selectedClouds[i]->id.c_str());
        }
    } else if(ui->cbox_feature_type->currentIndex()==3){//shot
        if(shots.empty()||selectedClouds.size()!=shots.size()){
            this->preview();
        }
        for(size_t i=0;i<selectedClouds.size();i++){
            selectedClouds[i]->setFeature(shots[i]);
            console->info(tr("Applied the shot feature to pointcloud of ")+selectedClouds[i]->id.c_str());
        }
    }
}

void Descriptor::reset()
{
    plotter.reset(new pcl::visualization::PCLPlotter("Histogram"));
    plotter->close();
    plotter=nullptr;
    fpfhs.clear();
    shots.clear();
}

void Descriptor::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QDockWidget::closeEvent(event);
}

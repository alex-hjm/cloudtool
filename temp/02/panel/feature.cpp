#include "feature.h"
#include "ui_feature.h"

Feature::Feature(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Feature),plotter(nullptr),pfh(new PFHFeature),
    fpfh(new FPFHFeature),vfh(new VFHFeature),shot(new SHOTFeature)
{
    ui->setupUi(this);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Feature::preview);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Feature::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Feature::reset);
    //feature
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
    //lrf
    connect(ui->rbtn_r1,&QRadioButton::clicked,[=](bool checked){
        if(checked) {
            ui->dspin_r1->setEnabled(true);
            ui->spin_k1->setEnabled(false);
            ui->spin_k1->setValue(0);
        }
    });
    connect(ui->rbtn_k1,&QRadioButton::clicked,[=](bool checked){
        if(checked) {
            ui->spin_k1->setEnabled(true);
            ui->dspin_r1->setEnabled(false);
            ui->dspin_r1->setValue(0);
        }
    });

    connect(ui->tabWidget,&QTabWidget::currentChanged,[=](int index)
    {
        switch(index)
        {
        case 0:
            ui->btn_preview->setEnabled(true);
            break;
        case 1:
            ui->btn_preview->setEnabled(false);
            break;
        }
    });


    connect(ui->cbox_lrf_type,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        switch (index) {
        case 0://board
            ui->dspin_tang_r->show();
            ui->dspin_margin_r->show();
            ui->check_findholes->show();
            break;
        case 1://flare
            ui->dspin_tang_r->show();
            ui->dspin_margin_r->show();
            ui->check_findholes->hide();
            break;
        case 2://shot
            ui->dspin_tang_r->hide();
            ui->dspin_margin_r->hide();
            ui->check_findholes->hide();
            break;
        }

    });
    ui->cbox_lrf_type->setCurrentIndex(0);
    ui->tabWidget->setCurrentIndex(0);
}

Feature::~Feature()
{
    delete ui;
}

void Feature::init()
{

}

void Feature::preview()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(ui->tabWidget->currentIndex()==1)return;
    int feature_type=ui->cbox_feature_type->currentIndex();
    if(feature_type==1) fpfhs.clear();
    if(feature_type==3) shots.clear();
    plotter.reset(new pcl::visualization::PCLPlotter("Histogram"));
    std::vector<double> array_x(500), array_y(500);
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(!selectedClouds[i].hasNormals) {
            console->warning(tr("Please Estimation the normals of pointcloud first!"));
            continue;
        }
        if(ui->check_keypoints->isChecked()&&(!selectedClouds[i].hasKeypts())){
            console->warning(tr("Please Estimation the keypoints of pointcloud first!"));
            continue;
        }
        switch (feature_type){
        case 0://PFHEstimation
            pfh.reset(new PFHFeature);
            plotter->setTitle("Point Feature Histogram");
            if(ui->check_keypoints->isChecked())
                pfh=feature.PFHEstimation(selectedClouds[i].keypoints,selectedClouds[i].cloud,ui->spin_k->value(),ui->dspin_r->value());
            else
                pfh=feature.PFHEstimation(selectedClouds[i].cloud,selectedClouds[i].cloud,ui->spin_k->value(),ui->dspin_r->value());
            if(!pfh->empty())
                plotter->addFeatureHistogram(*pfh,500);
            break;
        case 1://FPFHEstimation
            fpfh.reset(new FPFHFeature);
            plotter->setTitle("Fast Point Feature Histogram");
            if(ui->check_keypoints->isChecked())
                fpfh=feature.FPFHEstimation(selectedClouds[i].keypoints,selectedClouds[i].cloud,ui->spin_k->value(),ui->dspin_r->value());
            else
                fpfh=feature.FPFHEstimation(selectedClouds[i].cloud,selectedClouds[i].cloud,ui->spin_k->value(),ui->dspin_r->value());
            if(!fpfh->empty()) {
                fpfhs.push_back(fpfh);
                plotter->addFeatureHistogram(*fpfh,500);
            }
            break;
        case 2://VFHEstimation
            vfh.reset(new VFHFeature);
            plotter->setTitle("Viewpoint Feature Histogram");
            if(ui->check_keypoints->isChecked())
                vfh=feature.VFHEstimation(selectedClouds[i].keypoints,selectedClouds[i].cloud,ui->spin_k->value(),ui->dspin_r->value());
            else
                vfh=feature.VFHEstimation(selectedClouds[i].cloud,selectedClouds[i].cloud,ui->spin_k->value(),ui->dspin_r->value());
            if(!fpfh->empty())
                plotter->addFeatureHistogram(*vfh,500);
            break;
        case 3://SHOTEstimation
            shot.reset(new SHOTFeature);
            if(ui->check_keypoints->isChecked())
                shot=feature.SHOTEstimation(selectedClouds[i].keypoints,selectedClouds[i].cloud,ui->spin_k->value(),ui->dspin_r->value());
            else
                shot=feature.SHOTEstimation(selectedClouds[i].cloud,selectedClouds[i].cloud,ui->spin_k->value(),ui->dspin_r->value());
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
        if(ui->check_keypoints->isChecked())
            console->info(tr("The keypoints of ")+selectedClouds[i].id.c_str()+tr(" has calculated features,take time %1 ms").arg(feature.tocTime));
        else
            console->info(tr("The pointcloud of ")+selectedClouds[i].id.c_str()+tr(" has calculated features,take time %1 ms").arg(feature.tocTime));
    }
    plotter->setShowLegend (false);
    plotter->setWindowPosition((QApplication::desktop()->width()-plotter->getWindowSize()[0])/2,
            (QApplication::desktop()->height()-plotter->getWindowSize()[1])/2);
    plotter->plot();
}

void Feature::apply()
{
    std::vector<Cloud> selectedClouds=cloudTree->getSelectedClouds();
    if(selectedClouds.size()<=0) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    std::vector<Index> indexs=cloudTree->getSelectedIndexs();
    for(size_t i=0;i<selectedClouds.size();i++) {
        if(ui->tabWidget->currentIndex()==0){//feature
            if(ui->cbox_feature_type->currentIndex()==1) {//fpfh
                if((fpfhs.size()<=0)|(selectedClouds.size()!=fpfhs.size())){
                    this->preview();
                }
                *selectedClouds[i].fpfhFeature=*fpfhs[i];
                cloudTree->updateCloud(indexs[i],selectedClouds[i]);
                if(ui->check_keypoints->isChecked())
                    console->info(tr("Applied the FPFHFeature to keypoints of ")+selectedClouds[i].id.c_str());
                else
                    console->info(tr("Applied the FPFHFeature to pointcloud of ")+selectedClouds[i].id.c_str());
            } else if(ui->cbox_feature_type->currentIndex()==3){//shot
                if((shots.size()<=0)|(selectedClouds.size()!=shots.size())){
                    this->preview();
                }
                *selectedClouds[i].shotFeature=*shots[i];
                cloudTree->updateCloud(indexs[i],selectedClouds[i]);
                if(ui->check_keypoints->isChecked())
                    console->info(tr("Applied the SHOTFeature to keypoints of ")+selectedClouds[i].id.c_str());
                else
                    console->info(tr("Applied the SHOTFeature to pointcloud of ")+selectedClouds[i].id.c_str());
            }
        }else if(ui->tabWidget->currentIndex()==1) {//lrf
            if(!selectedClouds[i].hasNormals) {
                console->warning(tr("Please Estimation the normals of pointcloud first!"));
                continue;
            }
            if(ui->check_keypoints->isChecked()&&(!selectedClouds[i].hasKeypts())){
                console->warning(tr("Please Estimation the keypoints of pointcloud first!"));
                continue;
            }
            ReferenceFrame::Ptr lrf(new ReferenceFrame);
            switch (ui->cbox_lrf_type->currentIndex()){
            case 0://BOARD
                if(ui->check_keypoints->isChecked())
                    lrf=feature.BOARDLocalReferenceFrameEstimation(selectedClouds[i].keypoints,selectedClouds[i].cloud,ui->spin_k1->value(),ui->dspin_r1->value(),
                                                                   ui->dspin_tang_r->value(),ui->dspin_margin_r->value(),ui->check_findholes->isChecked());
                else
                    lrf=feature.BOARDLocalReferenceFrameEstimation(selectedClouds[i].cloud,selectedClouds[i].cloud,ui->spin_k1->value(),ui->dspin_r1->value(),
                                                                   ui->dspin_tang_r->value(),ui->dspin_margin_r->value(),ui->check_findholes->isChecked());
                break;
            case 1://FLARE
                if(ui->check_keypoints->isChecked())
                    lrf=feature.FLARELocalReferenceFrameEstimation(selectedClouds[i].keypoints,selectedClouds[i].cloud,ui->spin_k1->value(),ui->dspin_r1->value(),
                                                                   ui->dspin_tang_r->value(),ui->dspin_margin_r->value());
                else
                    lrf=feature.FLARELocalReferenceFrameEstimation(selectedClouds[i].cloud,selectedClouds[i].cloud,ui->spin_k->value(),ui->dspin_r->value(),
                                                                   ui->dspin_tang_r->value(),ui->dspin_margin_r->value());
                break;
            case 2://SHOT
                if(ui->check_keypoints->isChecked())
                    lrf=feature.SHOTLocalReferenceFrameEstimation(selectedClouds[i].keypoints,selectedClouds[i].cloud,ui->spin_k1->value(),ui->dspin_r1->value());
                else
                    lrf=feature.SHOTLocalReferenceFrameEstimation(selectedClouds[i].cloud,selectedClouds[i].cloud,ui->spin_k1->value(),ui->dspin_r1->value());
                break;
            }
            if(!lrf->empty()) {
                *selectedClouds[i].referenceframe=*lrf;
                cloudTree->updateCloud(indexs[i],selectedClouds[i]);
                if(ui->check_keypoints->isChecked())
                    console->info(tr("Applied the LocalReferenceFrame to keypoints of ")+selectedClouds[i].id.c_str());
                else
                    console->info(tr("Applied the LocalReferenceFrame to pointcloud of ")+selectedClouds[i].id.c_str());
            }
        }
    }
}

void Feature::reset()
{
    plotter.reset(new pcl::visualization::PCLPlotter("Histogram"));
    plotter->close();
    fpfhs.clear();
    shots.clear();
    plotter=nullptr;
}

void Feature::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QWidget::closeEvent(event);
}

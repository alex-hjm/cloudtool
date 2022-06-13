#include "correspondence.h"
#include "ui_correspondence.h"

Correspondence::Correspondence(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Correspondence),
    all_correspondence(new Correspondences),
    remain_correspondences(new Correspondences)
{
    ui->setupUi(this);
    connect(ui->btn_setTarget,&QPushButton::clicked,this,&Correspondence::setTarget);
    connect(ui->btn_setSource,&QPushButton::clicked,this,&Correspondence::setSource);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Correspondence::preview);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Correspondence::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Correspondence::reset);
    connect(ui->rbtn_ce,&QRadioButton::clicked,[=](bool checked)
    {
        if(checked){
            ui->stackedWidget->hide();
            ui->stackedWidget_2->show();
        }
    });
    connect(ui->rbtn_cr,&QRadioButton::clicked,[=](bool checked)
    {
        if(checked){
            ui->stackedWidget_2->hide();
            ui->stackedWidget->show();
        }
    });
    ui->rbtn_ce->setChecked(true);
    ui->stackedWidget->hide();
    ui->stackedWidget_2->show();
    ui->cbox_ce->setCurrentIndex(0);
    ui->cbox_cr->setCurrentIndex(0);
    ui->stackedWidget_2->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
}

Correspondence::~Correspondence()
{
    delete ui;
}

void Correspondence::init()
{
    cloudTree->setExtendedSelection(false);
}

void Correspondence::setTarget()
{
    Cloud selectedCloud=cloudTree->getSelectedCloud();
    if(selectedCloud.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((!sourceCloud.empty())&&(sourceCloud.id==selectedCloud.id)) {
        console->warning(tr("Please choose another pointcloud as target cloud!"));
        return;
    }
    targetCloud=selectedCloud;
    cloudView->setCloudColor(targetCloud.cloud,targetCloud.id,255,0,0);
    cloudView->removeShape(targetCloud.boxid);
    cloudView->showInfoText("Target Cloud : Red Cloud ",30,"info");
    ui->btn_setTarget->setEnabled(false);
}

void Correspondence::setSource()
{
    Cloud selectedCloud=cloudTree->getSelectedCloud();
    if(selectedCloud.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((!targetCloud.empty())&&(targetCloud.id==selectedCloud.id)) {
        console->warning(tr("Please choose another pointcloud as source cloud!"));
        return;
    }
    sourceCloud=selectedCloud;
    cloudView->setCloudColor(sourceCloud.cloud,sourceCloud.id,0,255,0);
    cloudView->removeShape(sourceCloud.boxid);
    cloudView->showInfoText("Source Cloud : Green Cloud ",50,"info1");
    ui->btn_setSource->setEnabled(false);
}

void Correspondence::preview()
{
    if(targetCloud.empty()|targetCloud.empty()) {
        console->warning(tr("Please select a source cloud and a target cloud!"));
        return;
    }
    //CorrespondenceEstimation
    if(ui->rbtn_ce->isChecked()) {
        int index=ui->cbox_ce->currentIndex();
        switch (index) {
        case 0://cloud
            all_correspondence=reg.CorrespondenceEstimation(targetCloud.cloud,sourceCloud.cloud);
            cloudView->showInfoText("pointcloud",70,"info2");
            break;
        case 1://fpfh feature
            if(targetCloud.fpfh_feature->empty()|sourceCloud.fpfh_feature->empty()) {
                console->warning(tr("Please Estimation the fpfh feature of pointcloud first!"));
                return;
            }
            all_correspondence=reg.CorrespondenceEstimation(targetCloud.fpfh_feature,sourceCloud.fpfh_feature);
            cloudView->showInfoText("fpfh feature",70,"info2");
            break;
        case 2://shot feature
            if(targetCloud.shot_feature->empty()|sourceCloud.shot_feature->empty()) {
                console->warning(tr("Please Estimation the shot feature of pointcloud first!"));
                return;
            }
            all_correspondence=reg.CorrespondenceEstimation(targetCloud.shot_feature,sourceCloud.shot_feature);
            cloudView->showInfoText("shot feature",70,"info2");
            break;
        case 3://FPFHKDTree
            if(targetCloud.fpfh_feature->empty()|sourceCloud.fpfh_feature->empty()) {
                console->warning(tr("Please Estimation the fpfh feature of pointcloud first!"));
                return;
            }
            all_correspondence=reg.CorrespondenceEstimationFPFHKDTree(targetCloud.fpfh_feature,sourceCloud.fpfh_feature,
                                                                      ui->dspin_thres1->value());
            cloudView->showInfoText("FPFHKDTree",70,"info2");
            break;
        case 4://SHOTKDTree
            if(targetCloud.shot_feature->empty()|sourceCloud.shot_feature->empty()) {
                console->warning(tr("Please Estimation the shot feature of pointcloud first!"));
                return;
            }
            all_correspondence=reg.CorrespondenceEstimationSHOTKDTree(targetCloud.shot_feature,sourceCloud.shot_feature,
                                                                      ui->dspin_thres2->value());
            cloudView->showInfoText("SHOTKDTree",70,"info2");
            break;
        case 5://Backprojection
            if(ui->check_keypoints->isChecked())
                all_correspondence=reg.CorrespondenceEstimationBackprojection(targetCloud.keypoints,sourceCloud.keypoints,
                                                                              ui->spin_k1->value());
            else
                all_correspondence=reg.CorrespondenceEstimationBackprojection(targetCloud.cloud,sourceCloud.cloud,
                                                                              ui->spin_k1->value());
            cloudView->showInfoText("Backprojection",70,"info2");
            break;
        case 6://NormalShooting
            if(ui->check_keypoints->isChecked())
                all_correspondence=reg.CorrespondenceEstimationNormalShooting(targetCloud.keypoints,sourceCloud.keypoints,
                                                                              ui->spin_k2->value());
            else
                all_correspondence=reg.CorrespondenceEstimationNormalShooting(targetCloud.cloud,sourceCloud.cloud,
                                                                              ui->spin_k2->value());
            cloudView->showInfoText("NormalShooting",70,"info2");
            break;
        }
        if(all_correspondence->size()<=0) {
            cloudView->removeCorrespondences("corres");
            return;
        }
        if(ui->check_keypoints->isChecked())
            cloudView->updateCorrespondences(sourceCloud.keypoints,targetCloud.keypoints,all_correspondence,"corres");
        else
            cloudView->updateCorrespondences(sourceCloud.cloud,targetCloud.cloud,all_correspondence,"corres");
    }
    else if(ui->rbtn_cr->isChecked()) {
        int index=ui->cbox_cr->currentIndex();
        if(all_correspondence->size()<=0) {
            console->warning(tr("Please Estimation the correspondences first!"));
            return;
        }
        if(ui->check_keypoints->isChecked())
            switch (index) {
            case 0://Distance
                remain_correspondences=reg.CorrespondenceRejectorDistance(sourceCloud.keypoints,targetCloud.keypoints,all_correspondence,
                                                                          ui->dspin_max_dis->value());
                break;
            case 1://MedianDistance
                remain_correspondences=reg.CorrespondenceRejectorMedianDistance(sourceCloud.keypoints,targetCloud.keypoints,all_correspondence,
                                                                                ui->dspin_factor->value());
                break;
            case 2://OneToOne
                remain_correspondences=reg.CorrespondenceRejectorOneToOne(sourceCloud.keypoints,targetCloud.keypoints,all_correspondence);
                break;
            case 3://OrganizedBoundary
                remain_correspondences=reg.CorrespondenceRejectionOrganizedBoundary(sourceCloud.keypoints,targetCloud.keypoints,all_correspondence,
                                                                                    ui->spin_val->value());
                break;
            case 4://Poly
                remain_correspondences=reg.CorrespondenceRejectorPoly(sourceCloud.keypoints,targetCloud.keypoints,all_correspondence,
                                                                      ui->spin_cardinality->value(),ui->dspin_sim_thres->value(),
                                                                      ui->spin_iterations->value());
                break;
            case 5://SampleConsensus
                remain_correspondences=reg.CorrespondenceRejectorSampleConsensus(sourceCloud.keypoints,targetCloud.keypoints,all_correspondence,
                                                                                 ui->dspin_inlier_thres->value(),ui->spin_max_iterations->value(),
                                                                                 ui->check_refine->isChecked(),ui->check_save->isChecked());
                break;
            case 6://SurfaceNormal
                remain_correspondences=reg.CorrespondenceRejectorSurfaceNormal(sourceCloud.keypoints,targetCloud.keypoints,all_correspondence,
                                                                               ui->dspin_threshold->value());
                break;
            case 7://Trimmed
                remain_correspondences=reg.CorrespondenceRejectorTrimmed(sourceCloud.keypoints,targetCloud.keypoints,all_correspondence,
                                                                         ui->dspin_ratio->value(),ui->spin_min_corre->value());
                break;
            case 8://VarTrimmed
                remain_correspondences=reg.CorrespondenceRejectoVarTrimmed(sourceCloud.keypoints,targetCloud.keypoints,all_correspondence,
                                                                           ui->dspin_min_ratio->value(),ui->dspin_max_ratio->value());
                break;
            } else {
            switch (index) {
            case 0://Distance
                remain_correspondences=reg.CorrespondenceRejectorDistance(sourceCloud.cloud,targetCloud.cloud,all_correspondence,
                                                                          ui->dspin_max_dis->value());
                break;
            case 1://MedianDistance
                remain_correspondences=reg.CorrespondenceRejectorMedianDistance(sourceCloud.cloud,targetCloud.cloud,all_correspondence,
                                                                                ui->dspin_factor->value());
                break;
            case 2://OneToOne
                remain_correspondences=reg.CorrespondenceRejectorOneToOne(sourceCloud.cloud,targetCloud.cloud,all_correspondence);
                break;
            case 3://OrganizedBoundary
                remain_correspondences=reg.CorrespondenceRejectionOrganizedBoundary(sourceCloud.cloud,targetCloud.cloud,all_correspondence,
                                                                                    ui->spin_val->value());
                break;
            case 4://Poly
                remain_correspondences=reg.CorrespondenceRejectorPoly(sourceCloud.cloud,targetCloud.cloud,all_correspondence,
                                                                      ui->spin_cardinality->value(),ui->dspin_sim_thres->value(),
                                                                      ui->spin_iterations->value());
                break;
            case 5://SampleConsensus
                remain_correspondences=reg.CorrespondenceRejectorSampleConsensus(sourceCloud.cloud,targetCloud.cloud,all_correspondence,
                                                                                 ui->dspin_inlier_thres->value(),ui->spin_max_iterations->value(),
                                                                                 ui->check_refine->isChecked(),ui->check_save->isChecked());
                break;
            case 6://SurfaceNormal
                remain_correspondences=reg.CorrespondenceRejectorSurfaceNormal(sourceCloud.cloud,targetCloud.cloud,all_correspondence,
                                                                               ui->dspin_threshold->value());
                break;
            case 7://Trimmed
                remain_correspondences=reg.CorrespondenceRejectorTrimmed(sourceCloud.cloud,targetCloud.cloud,all_correspondence,
                                                                         ui->dspin_ratio->value(),ui->spin_min_corre->value());
                break;
            case 8://VarTrimmed
                remain_correspondences=reg.CorrespondenceRejectoVarTrimmed(sourceCloud.cloud,targetCloud.cloud,all_correspondence,
                                                                           ui->dspin_min_ratio->value(),ui->dspin_max_ratio->value());
            }
        }
        if(remain_correspondences->size()<=0) {
            cloudView->removeCorrespondences("corres");
            return;
        }
        if(ui->check_keypoints->isChecked())
            cloudView->updateCorrespondences(sourceCloud.keypoints,targetCloud.keypoints,remain_correspondences,"corres");
        else
            cloudView->updateCorrespondences(sourceCloud.cloud,targetCloud.cloud,remain_correspondences,"corres");
    }
}

void Correspondence::apply()
{
    Cloud selectedCloud=cloudTree->getSelectedCloud();
    if(selectedCloud.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
}

void Correspondence::reset()
{
    cloudView->removeShape("info");
    cloudView->removeShape("info1");
    cloudView->removeShape("info2");
    cloudView->removeCorrespondences("corres");
    if(!targetCloud.empty())
        cloudView->resetCloudColor(targetCloud.cloud,targetCloud.id);
    if(!sourceCloud.empty())
        cloudView->resetCloudColor(sourceCloud.cloud,sourceCloud.id);
    targetCloud.cloud.reset(new CloudXYZRGBN);
    sourceCloud.cloud.reset(new CloudXYZRGBN);
    ui->btn_setSource->setEnabled(true);
    ui->btn_setTarget->setEnabled(true);
}

void Correspondence::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QWidget::closeEvent(event);
}

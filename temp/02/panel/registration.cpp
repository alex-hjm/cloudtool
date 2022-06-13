#include "registration.h"
#include "ui_registration.h"

Registration::Registration(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Registration),
    ail_cloud(new CloudXYZRGBN),
    all_corr(new Correspondences),
    num_iterations(0)
{
    ui->setupUi(this);
    connect(ui->btn_setTarget,&QPushButton::clicked,this,&Registration::setTarget);
    connect(ui->btn_setSouce,&QPushButton::clicked,this,&Registration::setSource);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Registration::preview);
    connect(ui->btn_add,&QPushButton::clicked,this,&Registration::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Registration::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Registration::reset);
    ///CorrespondenceEstimation
    connect(ui->cbox_ce,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        switch (index) {
        case 0://empty
            ui->dspin_value->hide();
            break;
        case 1://pointcloud
            ui->dspin_value->hide();
            break;
        case 2://Backprojection
            ui->dspin_value->show();
            ui->dspin_value->setDecimals(0);
            ui->dspin_value->setPrefix("Ksearch: ");
            ui->dspin_value->setValue(10);
            break;
        case 3://NormalShooting
            ui->dspin_value->show();
            ui->dspin_value->setDecimals(0);
            ui->dspin_value->setPrefix("Ksearch: ");
            ui->dspin_value->setValue(10);
            break;
        case 4://FPFHFeature
            ui->dspin_value->hide();
            break;
        case 5://SHOTFeature
            ui->dspin_value->hide();
            break;
        case 6://FPFHKDTree
            ui->dspin_value->show();
            ui->dspin_value->setDecimals(3);
            ui->dspin_value->setPrefix("Threshold: ");
            ui->dspin_value->setValue(0.25);
            break;
        case 7://SHOTKDTree
            ui->dspin_value->show();
            ui->dspin_value->setDecimals(3);
            ui->dspin_value->setPrefix("Threshold: ");
            ui->dspin_value->setValue(0.25);
            break;
        }
    });
    ui->dspin_value->hide();
    ui->cbox_cr->setCurrentIndex(0);
    ui->cbox_ce->setCurrentIndex(0);
    ui->stackedWidget_cr->setCurrentIndex(0);

    ///Registration
    connect(ui->check_continus,&QCheckBox::stateChanged,[=](int state)
    {
        if(state) num_iterations=ui->spin_nr_iterations->value();
    });
    connect(ui->cbox_registration,static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),[=](int index){
        ui->spin_nr_iterations->setValue(10);
        ui->spin_ransac_iterations->setValue(0);
        ui->dspin_inlier_threshold->setValue(0.05);
        switch (index) {
        case 0://IterativeClosestPoint
            ui->stackedWidget->setCurrentIndex(0);
            ui->spin_nr_samples->hide();
            ui->check_use_symmetric->hide();
            ui->check_enforce_samedirection->hide();
            break;
        case 1://IterativeClosestPointWithNormals
            ui->stackedWidget->setCurrentIndex(0);
            ui->spin_nr_samples->hide();
            ui->check_use_symmetric->show();
            ui->check_enforce_samedirection->show();
            ui->check_enforce_samedirection->setChecked(true);
            break;
        case 2://IterativeClosestPointNonLinear
            ui->stackedWidget->setCurrentIndex(0);
            ui->spin_nr_samples->hide();
            ui->check_use_symmetric->hide();
            ui->check_use_symmetric->hide();
            ui->check_enforce_samedirection->hide();
            ui->check_enforce_samedirection->hide();
            break;
        case 3://FPCSInitialAlignment
            ui->spin_nr_iterations->setValue(0);
            ui->spin_ransac_iterations->setValue(100);
            ui->stackedWidget->setCurrentIndex(1);
            ui->spin_nr_samples->show();
            break;
        case 4://KFPCSInitialAlignment
            ui->spin_nr_iterations->setValue(0);
            ui->spin_ransac_iterations->setValue(100);
            ui->stackedWidget->setCurrentIndex(2);
            ui->spin_nr_samples->show();
            break;
        case 5://SampleConsensusPrerejective
            ui->spin_ransac_iterations->setValue(3);
            ui->spin_nr_iterations->setValue(5000);
            ui->stackedWidget->setCurrentIndex(3);
            ui->spin_nr_samples->show();
            break;
        case 6://SampleConsensusInitialAlignment
            ui->spin_ransac_iterations->setValue(3);
            ui->spin_nr_iterations->setValue(1000);
            ui->dspin_distance_threshold->setValue(100);
            ui->stackedWidget->setCurrentIndex(4);
            ui->spin_nr_samples->show();
            break;
        case 7://NormalDistributionsTransform
            ui->spin_nr_iterations->setValue(35);
            ui->stackedWidget->setCurrentIndex(5);
            ui->spin_nr_samples->show();
            break;
        }
    });
    ui->cbox_registration->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
    ui->spin_nr_samples->hide();
    ui->check_use_symmetric->hide();
    ui->check_enforce_samedirection->hide();

    ui->toolBox->setCurrentIndex(1);
}

Registration::~Registration()
{
    delete ui;
}

void Registration::init()
{
    cloudTree->setExtendedSelection(false);
}

void Registration::setTarget()
{
    targetCloud=cloudTree->getSelectedCloud();
    if(targetCloud.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((!sourceCloud.empty())&&(sourceCloud.id==targetCloud.id)) {
        console->warning(tr("Please choose another pointcloud as target cloud!"));
        return;
    }
    cloudView->removeShape(targetCloud.boxid);
    targetIndex=cloudTree->getSelectedIndex();
    cloudView->setCloudColor(targetCloud.cloud,targetCloud.id,255,0,0);
    cloudView->showInfoText("Target Cloud : Red Cloud ",30,"info");
    ui->btn_setTarget->setEnabled(false);
    if(!sourceCloud.empty()&&!targetCloud.empty())
        cloudTree->setEnabled(false);
}

void Registration::setSource()
{
    sourceCloud=cloudTree->getSelectedCloud();
    if(sourceCloud.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((!targetCloud.empty())&&(targetCloud.id==sourceCloud.id)) {
        console->warning(tr("Please choose another pointcloud as source cloud!"));
        return;
    }
    cloudView->removeShape(sourceCloud.boxid);
    sourceIndex=cloudTree->getSelectedIndex();
    fileInfo=sourceCloud.fileInfo;
    cloudView->setCloudColor(sourceCloud.cloud,sourceCloud.id,0,255,0);
    cloudView->showInfoText("Source Cloud : Green Cloud ",50,"info1");
    ui->btn_setSouce->setEnabled(false);
    if(!sourceCloud.empty()&&!targetCloud.empty())
        cloudTree->setEnabled(false);
}

void Registration::preview()
{
    if(targetCloud.empty()|sourceCloud.empty()) {
        console->warning(tr("Please select a source cloud and a target cloud!"));
        return;
    }
    int ce_type=ui->cbox_ce->currentIndex();
    int cr_type=ui->cbox_cr->currentIndex();
    int te_type=ui->cbox_te->currentIndex();
    switch (ce_type) {
    case 0://empty
        break;
    case 1://pointcloud
        all_corr=reg.CorrespondenceEstimation(targetCloud.cloud,sourceCloud.cloud);
        break;
    case 2://Backprojection
        all_corr=reg.CorrespondenceEstimationBackprojection(targetCloud.cloud,sourceCloud.cloud,ui->dspin_value->value());
        break;
    case 3://NormalShooting
        all_corr=reg.CorrespondenceEstimationNormalShooting(targetCloud.cloud,sourceCloud.cloud,ui->dspin_value->value());
        break;
    case 4://FPFHFeature
        if(targetCloud.fpfhFeature->empty()|sourceCloud.fpfhFeature->empty()){
            console->warning(tr("Please estimation fpfh feature for the pointcloud first!"));
            return;
        }
        all_corr=reg.CorrespondenceEstimation(targetCloud.fpfhFeature,sourceCloud.fpfhFeature);
        break;
    case 5://SHOTFeature
        if(targetCloud.shotFeature->empty()|sourceCloud.shotFeature->empty()){
            console->warning(tr("Please estimation shot feature for the pointcloud first!"));
            return;
        }
        all_corr=reg.CorrespondenceEstimation(targetCloud.shotFeature,sourceCloud.shotFeature);
        break;
    case 6://FPFHKDTree
        if(targetCloud.fpfhFeature->empty()|sourceCloud.fpfhFeature->empty()){
            console->warning(tr("Please estimation fpfh feature for the pointcloud first!"));
            return;
        }
        all_corr=reg.CorrespondenceEstimationFPFHKDTree(targetCloud.fpfhFeature,sourceCloud.fpfhFeature,ui->dspin_value->value());
        break;
    case 7://SHOTKDTree
        if(targetCloud.shotFeature->empty()|sourceCloud.shotFeature->empty()){
            console->warning(tr("Please estimation shot feature for the pointcloud first!"));
            return;
        }
        all_corr=reg.CorrespondenceEstimationSHOTKDTree(targetCloud.shotFeature,sourceCloud.shotFeature,ui->dspin_value->value());
        break;
    }
    switch (cr_type) {
    case 0://empty
        break;
    case 1://Distance
        all_corr=reg.CorrespondenceRejectorDistance(targetCloud.cloud,sourceCloud.cloud,all_corr,ui->dspin_max_dis->value());
        break;
    case 2://MedianDistance
        all_corr=reg.CorrespondenceRejectorMedianDistance(targetCloud.cloud,sourceCloud.cloud,all_corr,ui->dspin_factor->value());
        break;
    case 3://OneToOne
        all_corr=reg.CorrespondenceRejectorOneToOne(all_corr);
        break;
    case 4://Polygon
        all_corr=reg.CorrespondenceRejectorPoly(targetCloud.cloud,sourceCloud.cloud,all_corr,ui->spin_card->value(),
                                                ui->dspin_sim_thre->value(),ui->spin_interat->value());
        break;
    case 5://SampleConsensus
        all_corr=reg.CorrespondenceRejectorSampleConsensus(targetCloud.cloud,sourceCloud.cloud,all_corr,ui->spin_max_iter->value(),
                                                           ui->dspin_inliner->value(),ui->check_refine->isChecked(),ui->check_save->isChecked());
        break;
    case 6://SurfaceNormal
        all_corr=reg.CorrespondenceRejectorSurfaceNormal(targetCloud.cloud,sourceCloud.cloud,all_corr,ui->dspin_thres->value());
        break;
    case 7://Trimmed
        all_corr=reg.CorrespondenceRejectorTrimmed(all_corr,ui->dspin_ratio->value(),ui->dspin_min_sample_distance->value());
        break;
    case 8://VarTrimmed
        all_corr=reg.CorrespondenceRejectorVarTrimmed(targetCloud.cloud,sourceCloud.cloud,all_corr,ui->dspin_min_ratio->value(),ui->dspin_max_ratio->value());
        break;
    }
    if(ui->toolBox->currentIndex()==0) {
        if((all_corr->size()<=0)) {
            cloudView->removeCorrespondences("all_corr");
        }
        cloudView->updateCorrespondences(sourceCloud.cloud,targetCloud.cloud,all_corr,"all_corr");
    } else if(ui->toolBox->currentIndex()==1) {
        if(ui->check_continus->isChecked()&!ail_cloud->empty()) {
            num_iterations+=ui->spin_nr_iterations->value();
            sourceCloud.cloud.reset(new CloudXYZRGBN);
            *sourceCloud.cloud=*ail_cloud;
        } else {
            num_iterations=ui->spin_nr_iterations->value();
        }
        ail_cloud.reset(new CloudXYZRGBN);
        switch (ui->cbox_registration->currentIndex())
        {
        case 0://IterativeClosestPoint
            ail_cloud=reg.IterativeClosestPoint(targetCloud.cloud,sourceCloud.cloud,te_type,ce_type,cr_type,ui->spin_nr_iterations->value(),
                                                ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                                ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                                ui->check_use_reciprocal->isChecked());
            cloudView->showInfoText("IterativeClosestPoint: Blue Cloud ",70,"info2");
            break;
        case 1://IterativeClosestPointWithNormals
            ail_cloud=reg.IterativeClosestPointWithNormals(targetCloud.cloud,sourceCloud.cloud,te_type,ce_type,cr_type,ui->spin_nr_iterations->value(),
                                                           ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                                           ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                                           ui->check_use_reciprocal->isChecked(),ui->check_use_symmetric->isChecked(),
                                                           ui->check_enforce_samedirection->isChecked());
            cloudView->showInfoText("IterativeClosestPointWithNormals: Blue Cloud ",70,"info2");
            break;
        case 2://IterativeClosestPointNonLinear
            ail_cloud=reg.IterativeClosestPointNonLinear(targetCloud.cloud,sourceCloud.cloud,te_type,ce_type,cr_type,ui->spin_nr_iterations->value(),
                                                         ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                                         ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                                         ui->check_use_reciprocal->isChecked());
            cloudView->showInfoText("IterativeClosestPointNonLinear: Blue Cloud ",70,"info2");
            break;
        case 3://FPCSInitialAlignment
            ail_cloud=reg.FPCSInitialAlignment(targetCloud.cloud,sourceCloud.cloud,te_type,ce_type,cr_type,ui->spin_nr_iterations->value(),
                                               ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                               ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                               ui->dspin_delta->value(),ui->dspin_approx_overlap->value(),ui->dspin_score_threshold->value(),
                                               ui->spin_nr_samples->value());
            cloudView->showInfoText("FPCSInitialAlignment: Blue Cloud ",70,"info2");
            break;
        case 4://KFPCSInitialAlignment
            ail_cloud=reg.KFPCSInitialAlignment(targetCloud.cloud,sourceCloud.cloud,te_type,ce_type,cr_type,ui->spin_nr_iterations->value(),
                                                ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                                ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                                ui->dspin_delta->value(),ui->dspin_approx_overlap->value(),ui->dspin_score_threshold->value(),
                                                ui->spin_nr_samples->value(),ui->dspin_upper_trl_boundary->value(),ui->dspin_lower_trl_boundary->value(),
                                                ui->dspin_lambda->value());
            cloudView->showInfoText("KFPCSInitialAlignment: Blue Cloud ",70,"info2");
            break;
        case 5://SampleConsensusPrerejective
            if(ui->cbox_feature->currentIndex()==0) {
                if(targetCloud.fpfhFeature->empty()|sourceCloud.fpfhFeature->empty()) {
                    console->warning(tr("Please Estimation the fpfh feature of pointcloud first!"));
                    return;
                }
                ail_cloud=reg.SampleConsensusPrerejective(targetCloud.cloud,sourceCloud.cloud,targetCloud.fpfhFeature,sourceCloud.fpfhFeature,te_type,ce_type,cr_type,
                                                          ui->spin_nr_iterations->value(),ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                                          ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                                          ui->spin_nr_samples->value(),ui->spin_k->value(),ui->dspin_similarity_threshold->value(),
                                                          ui->dspin_inlier_fraction->value());
            } else if(ui->cbox_feature->currentIndex()==1) {
                if(targetCloud.shotFeature->empty()|sourceCloud.shotFeature->empty()) {
                    console->warning(tr("Please Estimation the shot feature of pointcloud first!"));
                    return;
                }
                ail_cloud=reg.SampleConsensusPrerejective(targetCloud.cloud,sourceCloud.cloud,targetCloud.shotFeature,sourceCloud.shotFeature,te_type,ce_type,cr_type,
                                                          ui->spin_nr_iterations->value(),ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                                          ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                                          ui->spin_nr_samples->value(),ui->spin_k->value(),ui->dspin_similarity_threshold->value(),
                                                          ui->dspin_inlier_fraction->value());
            }
            cloudView->showInfoText("SampleConsensusPrerejective: Blue Cloud ",70,"info2");
            break;
        case 6://SampleConsensusInitialAlignment
            if(ui->cbox_feature2->currentIndex()==0) {
                if(targetCloud.fpfhFeature->empty()|sourceCloud.fpfhFeature->empty()) {
                    console->warning(tr("Please Estimation the feature of pointcloud first!"));
                    return;
                }
                ail_cloud=reg.SampleConsensusInitialAlignment(targetCloud.cloud,sourceCloud.cloud,targetCloud.fpfhFeature,sourceCloud.fpfhFeature,te_type,ce_type,cr_type,
                                                              ui->spin_nr_iterations->value(),ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                                              ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                                              ui->spin_nr_samples->value(),ui->spin_k2->value(),ui->dspin_min_sample_distance->value());
            }else if(ui->cbox_feature->currentIndex()==1) {
                if(targetCloud.shotFeature->empty()|sourceCloud.shotFeature->empty()) {
                    console->warning(tr("Please Estimation the shot feature of pointcloud first!"));
                    return;
                }
                ail_cloud=reg.SampleConsensusInitialAlignment(targetCloud.cloud,sourceCloud.cloud,targetCloud.shotFeature,sourceCloud.shotFeature,te_type,ce_type,cr_type,
                                                              ui->spin_nr_iterations->value(),ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                                              ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                                              ui->spin_nr_samples->value(),ui->spin_k2->value(),ui->dspin_min_sample_distance->value());
            }
            cloudView->showInfoText("SampleConsensusInitialAlignment: Blue Cloud ",70,"info2");
            break;
        case 7://NormalDistributionsTransform
            ail_cloud=reg.NormalDistributionsTransform(targetCloud.cloud,sourceCloud.cloud,ui->spin_nr_iterations->value(),te_type,ce_type,cr_type,
                                                       ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                                       ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                                       ui->dspin_resolution->value(),ui->dspin_step_size->value(),ui->dspin_outlier_ratio->value());
            cloudView->showInfoText("NormalDistributionsTransform: Blue Cloud ",70,"info2");
            break;
        }
        if(reg.score==-1){
            cloudView->removeCloud("reg");
            cloudView->removeShape("info2");
            console->warning(tr("Registration has not converged.Please retry!"));
            return;
        }
        console->info(tr("Applied %1 iteration to registration,take time: %2 ms").arg(num_iterations).arg(reg.tocTime));
        cloudView->updateCloud(ail_cloud,"reg");
        cloudView->setCloudColor(ail_cloud,"reg",0,0,255);
        ui->txt_matrix->clear();
        ui->txt_matrix->append(tr("Fitness Score: %1 ").arg(reg.score));
        cloudView->showInfoText("Iterations: "+std::to_string(num_iterations)+" Fitness Score: "+std::to_string(reg.score),90,"info3");
        ui->txt_matrix->append(tr("Transformation Matrix:"));
        ui->txt_matrix->append(tool.QStringFromMatrix(reg.matrix,6));
    }
}

void Registration::add()
{
    if(ail_cloud->empty()){
        console->warning(tr("Please register the pointcloud first!"));
        return;
    }
    Cloud cloud(ail_cloud,fileInfo);
    cloud.prefix("reg-");
    cloudTree->insertCloud(sourceIndex.row,cloud,true);
    cloudView->updateBoundingBox(cloud.box,cloud.boxid);
    this->reset();
}

void Registration::apply()
{
    if(targetCloud.empty()|sourceCloud.empty()) {
        console->warning(tr("Please select a source cloud and a target cloud!"));
        return;
    }
    if(ui->toolBox->currentIndex()==0) {
        if((all_corr->size()<=0))this->preview();
        *sourceCloud.correspondences=*all_corr;
        cloudTree->updateCloud(sourceIndex,sourceCloud);
        console->info(tr("Applied the correspondences to souce cloud ")+sourceCloud.id.c_str()+
                      " and target cloud"+targetCloud.id.c_str());
    } else {
        if(ail_cloud->empty())this->preview();
        *sourceCloud.cloud=*ail_cloud;
        cloudTree->updateCloud(sourceIndex,sourceCloud);
    }
    this->reset();
}

void Registration::reset()
{
    cloudView->removeShape("info");
    cloudView->removeShape("info1");
    cloudView->removeShape("info2");
    cloudView->removeShape("info3");
    cloudView->removeCloud("reg");
    cloudView->removeCorrespondences("all_corr");
    cloudView->removeCloud(targetCloud.id);
    cloudView->removeCloud(sourceCloud.id);
    num_iterations=0;
    ui->txt_matrix->clear();
    targetCloud.cloud.reset(new CloudXYZRGBN);
    sourceCloud.cloud.reset(new CloudXYZRGBN);
    ail_cloud.reset(new CloudXYZRGBN);
    ui->btn_setSouce->setEnabled(true);
    ui->btn_setTarget->setEnabled(true);
    cloudTree->setEnabled(true);
    std::vector<Index> indexs=cloudTree->getAllIndexs();
    for(int i=0 ;i<indexs.size();i++) {
        cloudTree->setCloudChecked(indexs[i],true);
    }
}

void Registration::closeEvent(QCloseEvent *event)
{
    this->reset();
    cloudTree->setExtendedSelection(true);
    return QWidget::closeEvent(event);
}

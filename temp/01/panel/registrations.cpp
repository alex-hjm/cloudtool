#include "registrations.h"
#include "ui_registrations.h"

Registrations::Registrations(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::Registrations),num_iterations(0),
    target_cloud(new Cloud),source_cloud(new Cloud),ail_cloud(new Cloud)
{
    ui->setupUi(this);
    connect(ui->btn_setTarget,&QPushButton::clicked,this,&Registrations::setTarget);
    connect(ui->btn_setSouce,&QPushButton::clicked,this,&Registrations::setSource);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Registrations::preview);
    connect(ui->btn_add,&QPushButton::clicked,this,&Registrations::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Registrations::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Registrations::reset);
    Registration *reg=new Registration;
    reg->moveToThread(&thread);
    qRegisterMetaType<FPFHFeature::Ptr>("FPFHFeature::Ptr &");
    qRegisterMetaType<FPFHFeature::Ptr>("FPFHFeature::Ptr");
    qRegisterMetaType<SHOTFeature::Ptr>("SHOTFeature::Ptr &");
    qRegisterMetaType<SHOTFeature::Ptr>("SHOTFeature::Ptr");
    qRegisterMetaType<Eigen::Matrix4f>("Eigen::Matrix4f &");
    qRegisterMetaType<Eigen::Matrix4f>("Eigen::Matrix4f");
    connect(&thread,&QThread::finished,reg,&QObject::deleteLater);
    connect(this,&Registrations::IterativeClosestPoint,reg,&Registration::IterativeClosestPoint);
    connect(this,&Registrations::IterativeClosestPointWithNormals,reg,&Registration::IterativeClosestPointWithNormals);
    connect(this,&Registrations::IterativeClosestPointNonLinear,reg,&Registration::IterativeClosestPointNonLinear);
    connect(this,&Registrations::NormalDistributionsTransform,reg,&Registration::NormalDistributionsTransform);
    connect(this,&Registrations::SampleConsensusPrerejectiveFPFH,reg,&Registration::SampleConsensusPrerejectiveFPFH);
    connect(this,&Registrations::SampleConsensusPrerejectiveSHOT,reg,&Registration::SampleConsensusPrerejectiveSHOT);
    connect(this,&Registrations::SampleConsensusInitialAlignmentFPFH,reg,&Registration::SampleConsensusInitialAlignmentFPFH);
    connect(this,&Registrations::SampleConsensusInitialAlignmentSHOT,reg,&Registration::SampleConsensusInitialAlignmentSHOT);
    connect(this,&Registrations::FPCSInitialAlignment,reg,&Registration::FPCSInitialAlignment);
    connect(this,&Registrations::KFPCSInitialAlignment,reg,&Registration::KFPCSInitialAlignment);
    connect(reg,&Registration::result,this,&Registrations::result);
    thread.start();

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
        case 3://NormalDistributionsTransform
            ui->stackedWidget->setCurrentIndex(1);
            ui->spin_nr_iterations->setValue(35);
            ui->spin_nr_samples->show();
            break;
        case 4://SampleConsensusPrerejective
            ui->stackedWidget->setCurrentIndex(2);
            ui->spin_ransac_iterations->setValue(3);
            ui->spin_nr_iterations->setValue(5000);

            ui->spin_nr_samples->show();
            break;
        case 5://SampleConsensusInitialAlignment
            ui->stackedWidget->setCurrentIndex(3);
            ui->spin_ransac_iterations->setValue(3);
            ui->spin_nr_iterations->setValue(1000);
            ui->dspin_distance_threshold->setValue(100);
            ui->spin_nr_samples->show();
            break;
        case 6://FPCSInitialAlignment
            ui->stackedWidget->setCurrentIndex(4);
            ui->spin_nr_iterations->setValue(0);
            ui->spin_ransac_iterations->setValue(100);
            ui->spin_nr_samples->show();
            break;
        case 7://KFPCSInitialAlignment
            ui->stackedWidget->setCurrentIndex(5);
            ui->spin_nr_iterations->setValue(0);
            ui->spin_ransac_iterations->setValue(100);
            ui->spin_nr_samples->show();
            break;
        }
    });
    ui->cbox_registration->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
    ui->spin_nr_samples->hide();
    ui->check_use_symmetric->hide();
    ui->check_enforce_samedirection->hide();
}


Registrations::~Registrations()
{
    thread.quit();
    thread.wait();
    delete ui;
}

void Registrations::init(Console* &co,CloudView* &cv,CloudTree* &ct)
{
    console=co;cloud_view=cv;cloud_tree=ct;
    cloud_tree->setExtendedSelection(false);
}

void Registrations::setTarget()
{
    target_cloud=cloud_tree->getSelectedCloud();
    if(target_cloud->empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(!source_cloud->empty()&&source_cloud->id==target_cloud->id) {
        console->warning(tr("Please choose another pointcloud as target cloud!"));
        return;
    }
    cloud_view->removeShape(target_cloud->box_id);
    target_index=cloud_tree->getSelectedIndex();
    cloud_view->setCloudColor(target_cloud,target_cloud->id,255,0,0);
    cloud_view->showInfo("Target Cloud : Red Cloud ",30,"info");
    ui->btn_setTarget->setEnabled(false);
    if(!source_cloud->empty()&&!target_cloud->empty())
        cloud_tree->setEnabled(false);
}

void Registrations::setSource()
{
    source_cloud=cloud_tree->getSelectedCloud();
    if(source_cloud->empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if(!target_cloud->empty()&&target_cloud->id==source_cloud->id) {
        console->warning(tr("Please choose another pointcloud as source cloud!"));
        return;
    }
    cloud_view->removeShape(source_cloud->box_id);
    source_index=cloud_tree->getSelectedIndex();
    cloud_view->setCloudColor(source_cloud,source_cloud->id,0,255,0);
    cloud_view->showInfo("Source Cloud : Green Cloud ",50,"info1");
    ui->btn_setSouce->setEnabled(false);
    if(!source_cloud->empty()&&!target_cloud->empty())
        cloud_tree->setEnabled(false);
}

void Registrations::preview()
{
    if(target_cloud->empty()||source_cloud->empty()) {
        console->warning(tr("Please select a source cloud and a target cloud!"));
        return;
    }
    if(ui->check_continus->isChecked()&&(!ail_cloud->empty())) {
        num_iterations+=ui->spin_nr_iterations->value();
        source_cloud=ail_cloud->makeShared();
    } else {
        num_iterations=ui->spin_nr_iterations->value();
    }
    ail_cloud.reset(new Cloud);
    switch (ui->cbox_registration->currentIndex())
    {
    case 0://IterativeClosestPoint
        emit IterativeClosestPoint(target_cloud,source_cloud,ui->spin_nr_iterations->value(),ui->spin_ransac_iterations->value(),
                                   ui->dspin_inlier_threshold->value(),ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                   ui->check_use_reciprocal->isChecked());
        cloud_view->showInfo("IterativeClosestPoint: Blue Cloud ",70,"info2");
        break;
    case 1://IterativeClosestPointWithNormals
        emit IterativeClosestPointWithNormals(target_cloud,source_cloud,ui->spin_nr_iterations->value(),ui->spin_ransac_iterations->value(),
                                              ui->dspin_inlier_threshold->value(),ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                              ui->check_use_reciprocal->isChecked(),ui->check_use_symmetric->isChecked(),
                                              ui->check_enforce_samedirection->isChecked());
        cloud_view->showInfo("IterativeClosestPointWithNormals: Blue Cloud ",70,"info2");
        break;
    case 2://IterativeClosestPointNonLinear
        emit IterativeClosestPointNonLinear(target_cloud,source_cloud,ui->spin_nr_iterations->value(),
                                            ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                            ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                            ui->check_use_reciprocal->isChecked());
        cloud_view->showInfo("IterativeClosestPointNonLinear: Blue Cloud ",70,"info2");
        break;
    case 3://NormalDistributionsTransform
        emit NormalDistributionsTransform(target_cloud,source_cloud,ui->spin_nr_iterations->value(),
                                          ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                          ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                          ui->dspin_resolution->value(),ui->dspin_step_size->value(),ui->dspin_outlier_ratio->value());
        cloud_view->showInfo("NormalDistributionsTransform: Blue Cloud ",70,"info2");
        break;
    case 4://SampleConsensusPrerejective
        if(ui->cbox_feature->currentIndex()==0) {
            if(target_cloud->fpfh_feature->empty()||source_cloud->fpfh_feature->empty()) {
                console->warning(tr("Please Estimation the fpfh feature of pointcloud first!"));
                return;
            }
            emit SampleConsensusPrerejectiveFPFH(target_cloud,source_cloud,target_cloud->fpfh_feature,source_cloud->fpfh_feature,
                                             ui->spin_nr_iterations->value(),ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                             ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                             ui->spin_nr_samples->value(),ui->spin_k->value(),ui->dspin_similarity_threshold->value(),
                                             ui->dspin_inlier_fraction->value());
        } else if(ui->cbox_feature->currentIndex()==1) {
            if(target_cloud->shot_feature->empty()||source_cloud->shot_feature->empty()) {
                console->warning(tr("Please Estimation the shot feature of pointcloud first!"));
                return;
            }
            emit SampleConsensusPrerejectiveSHOT(target_cloud,source_cloud,target_cloud->shot_feature,source_cloud->shot_feature,
                                             ui->spin_nr_iterations->value(),ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                             ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                             ui->spin_nr_samples->value(),ui->spin_k->value(),ui->dspin_similarity_threshold->value(),
                                             ui->dspin_inlier_fraction->value());
        }
        cloud_view->showInfo("SampleConsensusPrerejective: Blue Cloud ",70,"info2");
        break;
    case 5://SampleConsensusInitialAlignment
        if(ui->cbox_feature2->currentIndex()==0) {
            if(target_cloud->fpfh_feature->empty()||source_cloud->fpfh_feature->empty()) {
                console->warning(tr("Please Estimation the feature of pointcloud first!"));
                return;
            }
            emit SampleConsensusInitialAlignmentFPFH(target_cloud,source_cloud,target_cloud->fpfh_feature,source_cloud->fpfh_feature,
                                                 ui->spin_nr_iterations->value(),ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                                 ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                                 ui->spin_nr_samples->value(),ui->spin_k2->value(),ui->dspin_min_sample_distance->value());
        }else if(ui->cbox_feature->currentIndex()==1) {
            if(target_cloud->shot_feature->empty()||source_cloud->shot_feature->empty()) {
                console->warning(tr("Please Estimation the shot feature of pointcloud first!"));
                return;
            }
            emit SampleConsensusInitialAlignmentSHOT(target_cloud,source_cloud,target_cloud->shot_feature,source_cloud->shot_feature,
                                                 ui->spin_nr_iterations->value(),ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                                 ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                                 ui->spin_nr_samples->value(),ui->spin_k2->value(),ui->dspin_min_sample_distance->value());
        }
        cloud_view->showInfo("SampleConsensusInitialAlignment: Blue Cloud ",70,"info2");
        break;

    case 6://FPCSInitialAlignment
        emit FPCSInitialAlignment(target_cloud,source_cloud,ui->spin_nr_iterations->value(),
                                  ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                  ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                  ui->dspin_delta->value(),ui->dspin_approx_overlap->value(),ui->dspin_score_threshold->value(),
                                  ui->spin_nr_samples->value());
        cloud_view->showInfo("FPCSInitialAlignment: Blue Cloud ",70,"info2");
        break;
    case 7://KFPCSInitialAlignment
        emit KFPCSInitialAlignment(target_cloud,source_cloud,ui->spin_nr_iterations->value(),
                                   ui->spin_ransac_iterations->value(),ui->dspin_inlier_threshold->value(),
                                   ui->dspin_distance_threshold->value(),ui->dspin_fitness_epsilon->value(),
                                   ui->dspin_delta->value(),ui->dspin_approx_overlap->value(),ui->dspin_score_threshold->value(),
                                   ui->spin_nr_samples->value(),ui->dspin_upper_trl_boundary->value(),ui->dspin_lower_trl_boundary->value(),
                                   ui->dspin_lambda->value());
        cloud_view->showInfo("KFPCSInitialAlignment: Blue Cloud ",70,"info2");
        break;
    }
    console->showProgressBar();
}

void Registrations::add()
{
    if(ail_cloud->empty()){
        console->warning(tr("Please register the pointcloud first!"));
        return;
    }
    ail_cloud->prefix("reg-");
    ail_cloud->update();
    cloud_tree->insertCloud(source_index.row,ail_cloud,true);
    cloud_view->updateCube(ail_cloud->box,ail_cloud->box_id);
    this->reset();
}

void Registrations::apply()
{
   if(ail_cloud->empty())
       this->preview();

   Cloud::Ptr source_cloud_=cloud_tree->getCloud(source_index);
   source_cloud_->swap(*ail_cloud);
   source_cloud_->update();
   cloud_view->updateCube(source_cloud_->box,source_cloud_->box_id);
    this->reset();
}

void Registrations::reset()
{
    cloud_view->removeShape("info");
    cloud_view->removeShape("info1");
    cloud_view->removeShape("info2");
    cloud_view->removeShape("info3");
    cloud_view->removeCloud("reg");
    cloud_view->removeCloud(target_cloud->id);
    cloud_view->removeCloud(source_cloud->id);
    num_iterations=0;
    ui->txt_matrix->clear();
    target_cloud.reset(new Cloud);
    source_cloud.reset(new Cloud);
    ail_cloud.reset(new Cloud);
    ui->btn_setSouce->setEnabled(true);
    ui->btn_setTarget->setEnabled(true);
    cloud_tree->setEnabled(true);
    std::vector<Index> indexs=cloud_tree->getAllIndexs();
    for(int i=0 ;i<indexs.size();i++)
        cloud_tree->setCloudChecked(indexs[i],true);
}

void Registrations::result(bool success, const Cloud::Ptr &cloud, const Eigen::Matrix4f &matrix, double score, float time)
{
    console->closeProgressBar();
    if(!success) {
        cloud_view->removeCloud("reg");
        cloud_view->removeShape("info2");
        console->warning(tr("Registration has not converged.Please retry!"));
        return;
    }
    ail_cloud=cloud;
    console->info(tr("Applied %1 iteration to registration,take time: %2 ms").arg(num_iterations).arg(time));
    cloud_view->addCloud(cloud,"reg");
    cloud_view->setCloudColor(cloud,"reg",0,0,255);
    ui->txt_matrix->clear();
    ui->txt_matrix->append(tr("Fitness Score: %1 ").arg(score));
    cloud_view->showInfo("Iterations: "+std::to_string(num_iterations)+" Fitness Score: "+std::to_string(score),90,"info3");
    ui->txt_matrix->append(tr("Transformation Matrix:"));
    ui->txt_matrix->append(Tool::QStringFromMatrix(matrix,6));
}

void Registrations::closeEvent(QCloseEvent *event)
{
    this->reset();
    cloud_tree->setExtendedSelection(true);
    return QDockWidget::closeEvent(event);
}

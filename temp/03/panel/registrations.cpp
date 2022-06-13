#include "registrations.h"
#include "ui_registrations.h"

Registrations::Registrations(QWidget *parent) :
    QDockWidget(parent),
    ui(new Ui::Registrations),target_cloud(new Cloud),ail_cloud(new Cloud)
{
    ui->setupUi(this);
    connect(ui->btn_setTemplate,&QPushButton::clicked,this,&Registrations::setTemplate);
    connect(ui->btn_setTarget,&QPushButton::clicked,this,&Registrations::setTarget);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Registrations::preview);
    connect(ui->btn_add,&QPushButton::clicked,this,&Registrations::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Registrations::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Registrations::reset);

    ui->cbox_registration->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
}


Registrations::~Registrations()
{
    delete ui;
}

void Registrations::init(Console* &co,CloudView* &cv,CloudTree* &ct,ProcessTree* pt)
{
    console=co;cloud_view=cv;cloud_tree=ct;process_tree=pt;process_tree=pt;
}

void Registrations::setTemplate()
{
    std::vector<Cloud::Ptr> all_clouds=cloud_tree->getAllClouds();
    for(auto &i:template_cloud)
        cloud_view->resetCloudColor(i,i->id);
    Template *template_dialog=new Template(this);
    template_dialog->setTemplate(all_clouds);
    template_dialog->setExtendedSelection(true);
    connect(template_dialog,&Template::getTemplate,[=](const std::vector<int> &index){
        template_cloud.clear();
        for(size_t i=0;i<index.size();i++) {
            Cloud::Ptr cloud=all_clouds[index[i]];
            cloud_view->removeShape(cloud->box_id);
            cloud_view->setCloudColor(cloud,cloud->id,0,255,0);
            template_cloud.push_back(cloud);
            console->info(tr("Select the cloud ")+cloud->id.c_str()+tr(" as template cloud"));
        }
        cloud_view->showInfo("Template : Green Cloud ",30,"info");
    });
    if(template_dialog->exec()==QDialog::Rejected) {
        console->warning(tr("Select the template cloud canceled!"));
        return;
    }
}

void Registrations::setTarget()
{
    std::vector<Cloud::Ptr> all_clouds=cloud_tree->getAllClouds();
    if(!target_cloud->empty())
        cloud_view->resetCloudColor(target_cloud,target_cloud->id);
    Template *template_dialog=new Template(this);
    template_dialog->setTemplate(all_clouds);
    template_dialog->setExtendedSelection(false);
    connect(template_dialog,&Template::getIndex,[=](int index){
        target_cloud=all_clouds[index];
        cloud_view->removeShape(target_cloud->box_id);
        cloud_view->setCloudColor(target_cloud,target_cloud->id,255,0,0);
        console->info(tr("Select the cloud ")+target_cloud->id.c_str()+tr(" as target cloud"));
        cloud_view->showInfo("Target : Red Cloud ",50,"info1");
    });
    if(template_dialog->exec()==QDialog::Rejected) {
        console->warning(tr("Select the target cloud canceled!"));
        return;
    }
}

void Registrations::preview()
{
    if(template_cloud.empty()||target_cloud->empty()) {
        console->warning(tr("Please select some template clouds or a target cloud!"));
        return;
    }
    for(auto &i: template_cloud)
        cloud_view->setCloudColor(i,i->id,0,255,0);
    cloud_view->setCloudColor(target_cloud,target_cloud->id,255,0,0);
    pcl::console::TicToc time;
    time.tic();
    switch (ui->cbox_registration->currentIndex())
    {
    case 0://SampleConsensusInitialAlignment
        final_result=Registration::SampleConsensusInitialAlignment(target_cloud,template_cloud,ui->dspin_normals_radius->value(),ui->dspin_fpfh_radius->value(),
                                                                   ui->spin_nr_iterations->value(),ui->dspin_max_corrd_dis->value(),ui->spin_nr_samples->value(),
                                                                   ui->spin_k_corrd->value(),ui->dspin_min_sample_distance->value());
        cloud_view->showInfo("Alignment : Blue Cloud ",70,"info2");
        break;
    case 1://SampleConsensusPrerejective
        final_result=Registration::SampleConsensusPrerejective(target_cloud,template_cloud,ui->dspin_normals_radius->value(),ui->dspin_fpfh_radius->value(),
                                                               ui->spin_nr_iterations->value(),ui->dspin_max_corrd_dis->value(),ui->spin_nr_samples->value(),
                                                               ui->spin_k_corrd->value(),ui->dspin_similarity_threshold->value(),ui->dspin_inlier_fraction->value());
        cloud_view->showInfo("SampleConsensusPrerejective: Blue Cloud ",70,"info2");
        break;
    }
    if(final_result.index==-1) {
        cloud_view->removeCloud("reg");
        cloud_view->removeShape("info2");
        cloud_view->removeShape("info3");
        console->warning(tr("Registration has not converged.Please retry!"));
        return;
    } else {
        pcl::transformPointCloud(*(template_cloud[final_result.index]),*ail_cloud,final_result.final_transformation);
        ail_cloud->copyInfo(template_cloud[final_result.index]);
        console->info(tr("Registration has converged,take time: %1 ms").arg(time.toc()));
        cloud_view->addCloud(ail_cloud,"reg");
        cloud_view->setCloudColor(ail_cloud,"reg",0,0,255);
        cloud_view->setCloudSize("reg",2);
        if(ui->check_show_path->isChecked()) {
            for(size_t i=0;i<template_cloud[final_result.index]->path_points.size();i++) {
                Eigen::Matrix4f path_point=final_result.final_transformation*template_cloud[final_result.index]->path_points[i];
                Eigen::Affine3f path_point_aff(path_point);
                Eigen::Vector3f xyz=path_point.topRightCorner(3, 1);
                cloud_view->addCoord(template_cloud[final_result.index]->resolution*40,path_point_aff,"path_"+std::to_string(i));
                cloud_view->addText3D("path_"+std::to_string(i),PointXYZRGBN(xyz[0],xyz[1],xyz[2],0,0,0),template_cloud[final_result.index]->resolution*10,0,255,255,"path_txt"+std::to_string(i));
            }
        }
        //cloud_view->showInfo("Fitness Score: "+std::to_string(final_result.fitness_score),90,"info3");
        ui->txt_output->clear();
        ui->txt_output->append(tr("Fitness Score: %1 ").arg(final_result.fitness_score));
        ui->txt_output->append(tr("Transformation Matrix:"));
        ui->txt_output->append(Tool::QStringFromMatrix(final_result.final_transformation,6));
    }
}

void Registrations::add()
{
    if(process_tree->enable()){
        Process::Ptr registration(new Process);
        registration->id="Registration";
        registration->type=process_registration;
        registration->icon=QIcon(":/icon/resource/icon/registration.svg");
        switch (ui->cbox_registration->currentIndex()) {
        case 0:
            registration->value.resize(8);
            registration->value[0]=ui->cbox_registration->currentIndex();
            registration->value[1]=ui->dspin_normals_radius->value();
            registration->value[2]=ui->dspin_fpfh_radius->value();
            registration->value[3]=ui->spin_nr_iterations->value();
            registration->value[4]=ui->dspin_max_corrd_dis->value();
            registration->value[5]=ui->spin_nr_samples->value();
            registration->value[6]=ui->spin_k_corrd->value();
            registration->value[7]=ui->dspin_min_sample_distance->value();
            break;
        case 1:
            registration->value.resize(9);
            registration->value[0]=ui->cbox_registration->currentIndex();
            registration->value[1]=ui->dspin_normals_radius->value();
            registration->value[2]=ui->dspin_fpfh_radius->value();
            registration->value[3]=ui->spin_nr_iterations->value();
            registration->value[4]=ui->dspin_max_corrd_dis->value();
            registration->value[5]=ui->spin_nr_samples->value();
            registration->value[6]=ui->spin_k_corrd->value();
            registration->value[7]=ui->dspin_similarity_threshold->value();
            registration->value[8]=ui->dspin_inlier_fraction->value();
            break;
        }
        process_tree->insertProcess(registration);
        return;
    }
    if(ail_cloud->empty()) {
        console->warning(tr("Please registration the pointcloud first!"));
        return;
    }
    cloud_view->removeCloud("reg");
    cloud_view->removeShape("info2");
    cloud_view->removeShape("info3");
    Cloud::Ptr cloud=ail_cloud->makeShared();
    cloud->prefix("reg-");
    cloud->update();
    cloud_tree->insertCloud(-1,cloud,true);
    cloud_view->updateCube(cloud->box,cloud->box_id);
    console->info(tr("Added successfully!"));
    ail_cloud->clear();
}

void Registrations::apply()
{
    if(ail_cloud->empty()) {
        console->warning(tr("Please registration the pointcloud first!"));
        return;
    }
    cloud_view->removeCloud("reg");
    cloud_view->removeShape("info2");
    cloud_view->removeShape("info3");
    template_cloud[final_result.index]->swap(*ail_cloud);
    template_cloud[final_result.index]->update();
    cloud_view->updateCloud(template_cloud[final_result.index],template_cloud[final_result.index]->id);
    cloud_view->updateCube(template_cloud[final_result.index]->box,template_cloud[final_result.index]->box_id);
    console->info(tr("Applied successfully!"));
    ail_cloud->clear();
}

void Registrations::reset()
{
    cloud_view->removeCloud("reg");
    cloud_view->removeShape("info");
    cloud_view->removeShape("info1");
    cloud_view->removeShape("info2");
    cloud_view->removeShape("info3");
    ail_cloud->clear();
    if(!target_cloud->empty())
        cloud_view->resetCloudColor(target_cloud,target_cloud->id);
    target_cloud.reset(new Cloud);
    for(auto &i:template_cloud)
        cloud_view->resetCloudColor(i,i->id);
    template_cloud.clear();
    final_result.index=-1;
}

void Registrations::closeEvent(QCloseEvent *event)
{
    this->reset();
    return QDockWidget::closeEvent(event);
}

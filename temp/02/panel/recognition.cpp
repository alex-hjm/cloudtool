#include "recognition.h"
#include "ui_recognition.h"

Recognition::Recognition(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Recognition)
{
    ui->setupUi(this);
    connect(ui->btn_model,&QPushButton::clicked,this,&Recognition::setModel);
    connect(ui->btn_scene,&QPushButton::clicked,this,&Recognition::setScene);
    connect(ui->btn_preview,&QPushButton::clicked,this,&Recognition::preview);
    connect(ui->btn_add,&QPushButton::clicked,this,&Recognition::add);
    connect(ui->btn_apply,&QPushButton::clicked,this,&Recognition::apply);
    connect(ui->btn_reset,&QPushButton::clicked,this,&Recognition::reset);
}

Recognition::~Recognition()
{
    delete ui;
}

void Recognition::init()
{
    cloudTree->setExtendedSelection(false);
}

void Recognition::setModel()
{
    modelCloud=cloudTree->getSelectedCloud();
    if(modelCloud.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((!sceneCloud.empty())&&(sceneCloud.id==modelCloud.id)) {
        console->warning(tr("Please choose another pointcloud as model cloud!"));
        return;
    }
    cloudView->removeShape(modelCloud.boxid);
    modelIndex=cloudTree->getSelectedIndex();
    cloudView->setCloudColor(modelCloud.cloud,modelCloud.id,0,255,0);
    cloudView->showInfoText("Model Cloud : Green Cloud ",50,"info1");
    ui->btn_model->setEnabled(false);
    if(!modelCloud.empty()&&!sceneCloud.empty())
        cloudTree->setEnabled(false);
}

void Recognition::setScene()
{
    sceneCloud=cloudTree->getSelectedCloud();
    if(sceneCloud.empty()) {
        console->warning(tr("Please select a pointcloud!"));
        return;
    }
    if((!modelCloud.empty())&&(modelCloud.id==sceneCloud.id)) {
        console->warning(tr("Please choose another pointcloud as scene cloud!"));
        return;
    }
    cloudView->removeShape(sceneCloud.boxid);
    sceneIndex=cloudTree->getSelectedIndex();
    cloudView->setCloudColor(sceneCloud.cloud,sceneCloud.id,255,0,0);
    cloudView->showInfoText("Scene Cloud : Red Cloud ",30,"info");
    ui->btn_scene->setEnabled(false);
    if(!modelCloud.empty()&&!sceneCloud.empty())
        cloudTree->setEnabled(false);
}

void Recognition::preview()
{
    if(modelCloud.empty()|sceneCloud.empty()) {
        console->warning(tr("Please select a source cloud and a target cloud!"));
        return;
    }
    switch (ui->cbox_type->currentIndex())
    {
    case 0://GeometricConsistencyGrouping
        if(ui->check_keypoints->isChecked())
            clusters=rec.GeometricConsistencyGrouping(modelCloud.keypoints,sceneCloud.keypoints,modelCloud.correspondences,
                                                      ui->spin_gcthres->value(),ui->dspin_gcsize->value());
        else
            clusters=rec.GeometricConsistencyGrouping(modelCloud.cloud,sceneCloud.cloud,modelCloud.correspondences,
                                                      ui->spin_gcthres->value(),ui->dspin_gcsize->value());
        break;
    case 1:
        if(ui->check_keypoints->isChecked())
            clusters=rec.Hough3DGrouping(modelCloud.keypoints,sceneCloud.keypoints,modelCloud.referenceframe,sceneCloud.referenceframe,
                                         modelCloud.correspondences,ui->dspin_houghthres->value(),ui->dspin_binsize->value(),
                                         ui->check_useinter->isChecked(),ui->check_usedis->isChecked());
        else
            clusters=rec.Hough3DGrouping(modelCloud.cloud,sceneCloud.cloud,modelCloud.referenceframe,sceneCloud.referenceframe,
                                         modelCloud.correspondences,ui->dspin_houghthres->value(),ui->dspin_binsize->value(),
                                         ui->check_useinter->isChecked(),ui->check_usedis->isChecked());
        break;
    }
    console->info(tr("Model instances found: %1").arg(clusters.translations.size()));
    for (std::size_t i = 0; i < clusters.translations.size(); ++i) {
        console->info(tr("   Instance  %1 :").arg(i + 1));
        console->info(tr("   Correspondences belonging to this instance:  %1").arg(clusters.clustered_corrs[i].size ()));

        Eigen::Matrix3f rotation = clusters.translations[i].block<3,3>(0, 0);
        Eigen::Vector3f translation = clusters.translations[i].block<3,1>(0, 3);
        console->info(tr("\n"));
        console->info(tr("           | %1 %2 %3 |").arg(rotation (0,0)).arg(rotation (0,1)).arg(rotation (0,2)));
        console->info(tr("       R = | %1 %2 %3 |").arg(rotation (1,0)).arg(rotation (1,1)).arg(rotation (1,2)));
        console->info(tr("           | %1 %2 %3 |").arg(rotation (2,0)).arg(rotation (2,1)).arg(rotation (2,2)));
        console->info(tr("\n"));
        console->info(tr("       t = < %1, %2, %3 >").arg(translation (0)).arg(translation (1)).arg(translation (2)));

        CloudXYZRGBN::Ptr rec_cloud(new CloudXYZRGBN);
        pcl::transformPointCloud (*modelCloud.cloud, *rec_cloud, clusters.translations[i]);
        cloudView->updateCloud(rec_cloud,"rec_cloud");
        cloudView->setCloudColor(rec_cloud,"rec_cloud",0,0,255);
        recClouds.push_back(rec_cloud);
    }
}

void Recognition::add()
{

}

void Recognition::apply()
{

}

void Recognition::reset()
{

}

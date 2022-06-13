#include "keypoints.h"

#include "ui_keypoints.h"

std::unordered_map<QString, pca::Cloud::Ptr> KeyPoints::keypoints_map;

KeyPoints::KeyPoints(QWidget *parent)
    : CustomDock(parent), ui(new Ui::KeyPoints) {
  ui->setupUi(this);

  connect(ui->btn_add, &QPushButton::clicked, this, &KeyPoints::add);
  connect(ui->btn_apply, &QPushButton::clicked, this, &KeyPoints::apply);
  connect(ui->btn_preview, &QPushButton::clicked, this, &KeyPoints::preview);
  connect(ui->btn_reset, &QPushButton::clicked, this, &KeyPoints::reset);

  keypoints = new pca::Keypoints;
  keypoints->moveToThread(&thread);
  connect(&thread, &QThread::finished, keypoints, &QObject::deleteLater);
  connect(this, &KeyPoints::HarrisKeypoint3D, keypoints,
          &pca::Keypoints::HarrisKeypoint3D);
  connect(this, &KeyPoints::ISSKeypoint3D, keypoints,
          &pca::Keypoints::ISSKeypoint3D);
  connect(this, &KeyPoints::SIFTKeypoint, keypoints,
          &pca::Keypoints::SIFTKeypoint);
  connect(this, &KeyPoints::TrajkovicKeypoint3D, keypoints,
          &pca::Keypoints::TrajkovicKeypoint3D);
  connect(keypoints, &pca::Keypoints::keypointsResult, this,
          &KeyPoints::keypointsResult);
  thread.start();

  ui->cbox_type->setCurrentIndex(0);
  ui->stackedWidget->setCurrentIndex(0);
}

KeyPoints::~KeyPoints() {
  thread.quit();
  thread.wait();
  delete ui;
}

void KeyPoints::init() {
  connect(cloudtree, &pca::CloudTree::removedCloudId, [=](const QString &id) {
    if (keypoints_map.find(id) != keypoints_map.end()) keypoints_map.erase(id);
    std::cout << keypoints_map.size();
  });
}

void KeyPoints::preview() {
  if (!checkValid(true)) return;
  for (auto &cloud : selected_clouds) {
    keypoints->setInputCloud(cloud);
    keypoints->setKSearch(ui->spin_k->value());
    keypoints->setRadiusSearch(ui->dspin_r->value());
    switch (ui->cbox_type->currentIndex()) {
      case 0:  // HarrisKeypoint3D
        cloudview->showInfo("HarrisKeypoint3D", 1);
        emit HarrisKeypoint3D(ui->cbox_response_method->currentIndex(),
                              (float)ui->dspin_radius->value(),
                              (float)ui->dspin_threshold->value(),
                              ui->check_non_maxima->isChecked(),
                              ui->check_do_refine->isChecked());
        break;
      case 1:  // ISSKeypoint3D
        cloudview->showInfo("ISSKeypoint3D", 1);
        emit ISSKeypoint3D(
            ui->dspin_salient_radius->value(),
            ui->dspin_non_max_radius->value(), ui->dspin_normal_radius->value(),
            ui->dspin_border_radius->value(), ui->dspin_gamma_21->value(),
            ui->dspin_gamma_32->value(), ui->spin_min_neighbors->value(),
            (float)ui->dspin_angle->value());
        break;
      case 2:  // SIFTKeypoint
        cloudview->showInfo("SIFTKeypoint", 1);
        emit SIFTKeypoint((float)ui->dspin_min_scale->value(),
                          ui->spin_nr_octaves->value(),
                          ui->spin_nr_scales_per_octave->value(),
                          (float)ui->dspin_min_contrast->value());
        break;
      case 3:  // TrajkovicKeypoint3D
        cloudview->showInfo("TrajkovicKeypoint3D", 1);
        emit TrajkovicKeypoint3D(ui->cbox_compute_method->currentIndex(),
                                 ui->spin_window_size->value(),
                                 (float)ui->dspin_first_threshold->value(),
                                 (float)ui->dspin_second_threshold->value());
        break;
    }
    cloudtree->showProgressBar();
  }
}

void KeyPoints::add() {
  if (!checkValid()) return;
  for (auto &cloud : selected_clouds) {
    cloudview->removePointCloud(cloud->id() + "_keypoints");
    pca::Cloud::Ptr new_cloud = keypoints_map_tmp.find(cloud->id())->second;
    new_cloud->setId("keypoints-" + cloud->id());
    new_cloud->update();
    cloudtree->appendCloud(cloud, new_cloud, true);
    keypoints_map_tmp.erase(cloud->id());
  }
  log(pca::LOG_INFO, "the keypoints added done!");
  cloudview->clearInfo();
}

void KeyPoints::apply() {
  if (!checkValid()) return;
  for (auto &cloud : selected_clouds) {
    cloudview->removePointCloud(cloud->id() + "_keypoints");
    pca::Cloud::Ptr new_cloud = keypoints_map_tmp.find(cloud->id())->second;
    keypoints_map[cloud->id()] = new_cloud;
    keypoints_map_tmp.erase(cloud->id());
  }
  log(pca::LOG_INFO, "the keypoints has applied done!");
  cloudview->clearInfo();
}

void KeyPoints::reset() {
  for (auto &keypoint : keypoints_map_tmp)
    cloudview->removePointCloud(keypoint.second->id());
  keypoints_map_tmp.clear();
  cloudview->clearInfo();
}

void KeyPoints::keypointsResult(const pca::Cloud::Ptr &cloud, float time) {
  if (cloud->size() == 0) {
    log(pca::LOG_ERROR,
        "the pointcloud " + cloud->id() +
            tr(" estimate keypoints failed,take time %1 ms.").arg(time));
    cloudtree->closeProgressBar();
    return;
  }
  keypoints_map_tmp[cloud->id()] = cloud;
  cloud->setId(cloud->id() + "_keypoints");
  cloudview->addPointCloud(cloud);
  cloudview->setPointCloudColor(cloud->id(), 0, 255, 0);
  cloudview->setPointCloudSize(cloud->id(), cloud->pointSize() + 2);
  log(pca::LOG_INFO,
      "the pointcloud " + cloud->id() +
          tr(" estimate keypoints done,take time %1 ms.").arg(time));
  cloudtree->closeProgressBar();
}

bool KeyPoints::checkValid(bool preview) {
  selected_clouds = cloudtree->getSelectedClouds();
  if (selected_clouds.empty()) {
    log(pca::LOG_WARNING, "please select a pointcloud!");
    return false;
  }
  if (preview) {
    if (ui->spin_k->value() == 0 && ui->dspin_r->value() == 0) {
      log(pca::LOG_WARNING, "parameters set error!");
      return false;
    }
  } else
    for (auto &cloud : selected_clouds)
      if (keypoints_map_tmp.find(cloud->id()) == keypoints_map_tmp.end()) {
        log(pca::LOG_WARNING,
            "please estimation the keypoints or select the correct "
            "pointcloud!");
        return false;
      }
  return true;
}

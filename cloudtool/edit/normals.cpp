#include "normals.h"

#include "ui_normals.h"

Normals::Normals(QWidget *parent)
    : CustomDock(parent), thread(this), ui(new Ui::Normals) {
  ui->setupUi(this);

  qRegisterMetaType<Eigen::Vector3f>("Eigen::Vector3f &");
  qRegisterMetaType<Eigen::Vector3f>("Eigen::Vector3f");

  connect(ui->btn_preview, &QPushButton::clicked, this, &Normals::preview);
  connect(ui->btn_add, &QPushButton::clicked, this, &Normals::add);
  connect(ui->btn_apply, &QPushButton::clicked, this, &Normals::apply);
  connect(ui->btn_reset, &QPushButton::clicked, this, &Normals::reset);

  feature = new pca::Features;
  feature->moveToThread(&thread);
  connect(&thread, &QThread::finished, feature, &QObject::deleteLater);
  connect(this, &Normals::NormalEstimation, feature,
          &pca::Features::NormalEstimation);
  connect(feature, &pca::Features::normalsResult, this,
          &Normals::normalsResult);
  thread.start();

  connect(ui->check_max, &QCheckBox::clicked, [=](bool state) {
    if (state) {
      ui->check_center->setChecked(false);
      ui->check_origin->setChecked(false);
    }
  });
  connect(ui->check_center, &QCheckBox::clicked, [=](bool state) {
    if (state) {
      ui->check_max->setChecked(false);
      ui->check_origin->setChecked(false);
    }
  });
  connect(ui->check_origin, &QCheckBox::clicked, [=](bool state) {
    if (state) {
      ui->check_max->setChecked(false);
      ui->check_center->setChecked(false);
    }
  });

  connect(ui->spin_level,
          static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
          [=](int) {
            if (ui->check_refresh->isChecked()) this->updateNormals();
          });
  connect(ui->dspin_scale,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double) {
            if (ui->check_refresh->isChecked()) this->updateNormals();
          });

  connect(ui->rbtn_k, &QRadioButton::clicked, [=](bool checked) {
    if (checked) {
      ui->spin_k->setEnabled(true);
      ui->dspin_r->setValue(0);
      ui->dspin_r->setEnabled(false);
    }
  });
  connect(ui->rbtn_r, &QRadioButton::clicked, [=](bool checked) {
    if (checked) {
      ui->dspin_r->setEnabled(true);
      ui->spin_k->setValue(0);
      ui->spin_k->setEnabled(false);
    }
  });
}

Normals::~Normals() {
  thread.quit();
  thread.wait();
  delete ui;
}

void Normals::init() {
  connect(ui->check_reverse, &QCheckBox::stateChanged, [=]() {
    if (!checkValid()) return;
    for (auto &cloud : selected_clouds) {
      pca::Cloud::Ptr normal = normals_map.find(cloud->id())->second;
      for (auto &point : normal->points) {
        point.normal_x = -point.normal_x;
        point.normal_y = -point.normal_y;
        point.normal_z = -point.normal_z;
      }
      cloudview->addPointCloudNormals(normal, ui->spin_level->value(),
                                      ui->dspin_scale->value());
    }
  });
}

void Normals::preview() {
  if (!checkValid(true)) return;
  if (ui->rbtn_k->isChecked())
    cloudview->showInfo("K-nearest neighbor search estimation", 1);
  else
    cloudview->showInfo("R-radius search estimation", 1);
  for (auto &cloud : selected_clouds) {
    feature->setInputCloud(cloud);
    feature->setKSearch(ui->spin_k->value());
    feature->setRadiusSearch(ui->dspin_r->value());
    Eigen::Vector3f viewpoint;
    if (ui->check_origin->isChecked())
      viewpoint << cloud->sensor_origin_.coeff(0),
          cloud->sensor_origin_.coeff(1), cloud->sensor_origin_.coeff(2);
    else if (ui->check_center->isChecked())
      viewpoint = cloud->center();
    else if (ui->check_max)
      viewpoint << std::numeric_limits<float>::max(),
          std::numeric_limits<float>::max(), std::numeric_limits<float>::max();
    emit NormalEstimation(viewpoint[0], viewpoint[1], viewpoint[2]);
    cloudtree->showProgressBar();
  }
}

void Normals::add() {
  if (!checkValid()) return;
  for (auto &cloud : selected_clouds) {
    cloudview->removePointCloud(cloud->normalId());
    pca::Cloud::Ptr new_cloud = normals_map.find(cloud->id())->second;
    new_cloud->setId("normals-" + new_cloud->id());
    new_cloud->update();
    cloudtree->appendCloud(cloud, new_cloud, true);
    normals_map.erase(cloud->id());
  }
  log(pca::LOG_INFO, "the normals has added done!");
  cloudview->clearInfo();
}

void Normals::apply() {
  if (!checkValid()) return;
  for (auto &cloud : selected_clouds) {
    cloudview->removePointCloud(cloud->normalId());
    pca::Cloud::Ptr new_cloud = normals_map.find(cloud->id())->second;
    cloud->swap(*new_cloud);
    cloud->update();
    normals_map.erase(cloud->id());
  }
  log(pca::LOG_INFO, "the normals has applied done!");
  cloudview->clearInfo();
}

void Normals::reset() {
  for (auto &normal : normals_map)
    cloudview->removePointCloud(normal.second->normalId());
  normals_map.clear();
  cloudview->clearInfo();
}

void Normals::normalsResult(const pca::Cloud::Ptr &cloud, float time) {
  if (ui->check_reverse->isChecked()) {
    for (auto &point : cloud->points) {
      point.normal_x = -point.normal_x;
      point.normal_y = -point.normal_y;
      point.normal_z = -point.normal_z;
    }
  }
  cloudview->addPointCloudNormals(cloud, ui->spin_level->value(),
                                  ui->dspin_scale->value());
  normals_map[cloud->id()] = cloud;
  log(pca::LOG_INFO,
      "the pointcloud " + cloud->id() +
          tr(" estimate normals done,take time %1 ms.").arg(time));
  cloudtree->closeProgressBar();
}

void Normals::updateNormals() {
  if (!checkValid()) return;
  for (auto &cloud : selected_clouds) {
    pca::Cloud::Ptr normal = normals_map.find(cloud->id())->second;
    cloudview->addPointCloudNormals(normal, ui->spin_level->value(),
                                    ui->dspin_scale->value());
  }
}

bool Normals::checkValid(bool preview) {
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
      if (normals_map.find(cloud->id()) == normals_map.end()) {
        log(pca::LOG_WARNING,
            "please estimation the normals or select the correct pointcloud!");
        return false;
      }
  return true;
}

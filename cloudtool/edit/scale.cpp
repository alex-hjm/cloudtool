#include "scale.h"

#include "ui_scale.h"

Scale::Scale(QWidget *parent) : CustomDialog(parent), ui(new Ui::Scale) {
  ui->setupUi(this);
  connect(ui->btn_add, &QPushButton::clicked, this, &Scale::add);
  connect(ui->btn_apply, &QPushButton::clicked, this, &Scale::apply);
  connect(ui->btn_reset, &QPushButton::clicked, this, &Scale::reset);
  connect(ui->btn_close, &QPushButton::clicked, this, &Scale::close);

  connect(ui->check_samevalue, &QCheckBox::stateChanged, [=](int state) {
    if (state) {
      ui->dspin_y->setEnabled(false);
      ui->dspin_z->setEnabled(false);
    } else {
      ui->dspin_y->setEnabled(true);
      ui->dspin_z->setEnabled(true);
    }
  });

  connect(ui->dspin_x,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            if (ui->check_samevalue->isChecked()) {
              ui->dspin_y->setValue(value);
              ui->dspin_z->setValue(value);
              emit scale(value, value, value);
            }
            emit scale(value, ui->dspin_y->value(), ui->dspin_z->value());
          });

  connect(ui->dspin_y,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            if (ui->check_samevalue->isChecked()) return;
            emit scale(ui->dspin_x->value(), value, ui->dspin_z->value());
          });

  connect(ui->dspin_z,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            if (ui->check_samevalue->isChecked()) return;
            emit scale(ui->dspin_x->value(), ui->dspin_y->value(), value);
          });
  ui->check_samevalue->setChecked(true);
}

Scale::~Scale() { delete ui; }

void Scale::init() { connect(this, &Scale::scale, this, &Scale::preview); }

void Scale::add() {
  if (!checkValid()) return;
  for (auto &cloud : selected_clouds) {
    cloudview->removePointCloud(cloud->id() + "_scale");
    pca::Cloud::Ptr new_cloud = scaled_clouds_map.find(cloud->id())->second;
    new_cloud->setId("scaled-" + cloud->id());
    new_cloud->update();
    cloudtree->appendCloud(cloud, new_cloud, true);
    scaled_clouds_map.erase(cloud->id());
  }
  log(pca::LOG_INFO, "the scaled clouds added done!");
  cloudview->clearInfo();
}

void Scale::apply() {
  if (!checkValid()) return;
  for (auto &cloud : selected_clouds) {
    cloudview->removePointCloud(cloud->id() + "_scale");
    pca::Cloud::Ptr new_cloud = scaled_clouds_map.find(cloud->id())->second;
    cloudview->removePointCloud(new_cloud->id());
    cloud->swap(*new_cloud);
    cloud->update();
    scaled_clouds_map.erase(cloud->id());
  }
  cloudtree->setSelectedCloudsChecked(true);
  log(pca::LOG_INFO, "the scaled clouds applied done!");
  cloudview->clearInfo();
}

void Scale::reset() {
  cloudtree->setSelectedCloudsChecked(true);
  for (auto &cloud : scaled_clouds_map)
    cloudview->removePointCloud(cloud.second->id());
  scaled_clouds_map.clear();
  cloudview->clearInfo();
}

void Scale::preview(double x, double y, double z) {
  if (!checkValid(true)) return;
  cloudtree->setSelectedCloudsChecked(false);
  cloudview->showInfo("Scale Pointcloud", 1);
  for (auto &cloud : selected_clouds) {
    pca::Cloud::Ptr scaled_cloud = cloud->makeShared();
    if (ui->cbox_type->currentIndex() == 0)
      scaled_cloud->scale(x, y, z, false);
    else if (ui->cbox_type->currentIndex() == 1)
      scaled_cloud->scale(x, y, z, true);
    if (ui->check_keepentity->isChecked()) cloudview->resetCamera();
    scaled_cloud->setId(scaled_cloud->id() + "_scale");
    cloudview->addPointCloud(scaled_cloud);
    scaled_clouds_map[cloud->id()] = scaled_cloud;
  }
}

bool Scale::checkValid(bool preview) {
  selected_clouds = cloudtree->getSelectedClouds();
  if (selected_clouds.empty()) {
    log(pca::LOG_WARNING, "please select a pointcloud!");
    return false;
  }
  if (!preview)
    for (auto &cloud : selected_clouds)
      if (scaled_clouds_map.find(cloud->id()) == scaled_clouds_map.end())
        this->preview(ui->dspin_x->value(), ui->dspin_y->value(),
                      ui->dspin_z->value());
  return true;
}

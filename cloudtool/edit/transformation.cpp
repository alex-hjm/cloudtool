#include "transformation.h"

#include "pca/common.h"
#include "ui_transformation.h"

Transformation::Transformation(QWidget *parent)
    : CustomDock(parent),
      ui(new Ui::Transformation),
      trans_affine(Eigen::Affine3f::Identity()) {
  ui->setupUi(this);
  connect(ui->btn_add, &QPushButton::clicked, this, &Transformation::add);
  connect(ui->btn_apply, &QPushButton::clicked, this, &Transformation::apply);
  connect(ui->btn_reset, &QPushButton::clicked, this, &Transformation::reset);
  connect(ui->btn_preview, &QPushButton::clicked,
          [=] { this->preview(trans_affine); });
  ui->tabWidget->setCurrentIndex(0);

  // matrix
  connect(ui->txt_matrix, &QTextEdit::textChanged, [=]() {
    if (!pca::getTransformation(ui->txt_matrix->toPlainText(), trans_affine)) {
      log(pca::LOG_WARNING, "The transformation matrix format is wrong");
      return;
    }
    if (ui->tabWidget->currentIndex() == 0) emit affine(trans_affine);
  });

  // eulerAngle
  connect(ui->dspin_rx1,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            trans_affine = pca::getTransformation(
                ui->dspin_tx1->value(), ui->dspin_ty1->value(),
                ui->dspin_tz1->value(), value, ui->dspin_ry1->value(),
                ui->dspin_rz1->value());
            if (ui->tabWidget->currentIndex() == 1) emit affine(trans_affine);
          });
  connect(ui->dspin_ry1,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            trans_affine = pca::getTransformation(
                ui->dspin_tx1->value(), ui->dspin_ty1->value(),
                ui->dspin_tz1->value(), ui->dspin_rx1->value(), value,
                ui->dspin_rz1->value());
            if (ui->tabWidget->currentIndex() == 1) emit affine(trans_affine);
          });
  connect(ui->dspin_rz1,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            trans_affine = pca::getTransformation(
                ui->dspin_tx1->value(), ui->dspin_ty1->value(),
                ui->dspin_tz1->value(), ui->dspin_rx1->value(),
                ui->dspin_ry1->value(), value);
            if (ui->tabWidget->currentIndex() == 1) emit affine(trans_affine);
          });
  connect(ui->dspin_tx1,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            trans_affine = pca::getTransformation(
                value, ui->dspin_ty1->value(), ui->dspin_tz1->value(),
                ui->dspin_rx1->value(), ui->dspin_ry1->value(),
                ui->dspin_rz1->value());
            if (ui->tabWidget->currentIndex() == 1) emit affine(trans_affine);
          });
  connect(ui->dspin_ty1,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            trans_affine = pca::getTransformation(
                ui->dspin_tx1->value(), value, ui->dspin_tz1->value(),
                ui->dspin_rx1->value(), ui->dspin_ry1->value(),
                ui->dspin_rz1->value());
            if (ui->tabWidget->currentIndex() == 1) emit affine(trans_affine);
          });
  connect(ui->dspin_tz1,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            trans_affine = pca::getTransformation(
                ui->dspin_tx1->value(), ui->dspin_ty1->value(), value,
                ui->dspin_rx1->value(), ui->dspin_ry1->value(),
                ui->dspin_rz1->value());
            if (ui->tabWidget->currentIndex() == 1) emit affine(trans_affine);
          });
  connect(ui->txt_xyzeuler, &QLineEdit::textChanged, [=](const QString &text) {
    if (!pca::getTransformation(text, trans_affine)) {
      log(pca::LOG_WARNING, "The transformation matrix format is wrong");
      return;
    }
    if (ui->tabWidget->currentIndex() == 0) emit affine(trans_affine);
  });

  // axisAngle
  connect(ui->dspin_angle,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            trans_affine = pca::getTransformation(
                value, ui->dspin_ax->value(), ui->dspin_ay->value(),
                ui->dspin_az->value(), ui->dspin_tx2->value(),
                ui->dspin_ty2->value(), ui->dspin_tz2->value());
            if (ui->tabWidget->currentIndex() == 2) emit affine(trans_affine);
          });
  connect(ui->dspin_ax,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            trans_affine = pca::getTransformation(
                ui->dspin_angle->value(), value, ui->dspin_ay->value(),
                ui->dspin_az->value(), ui->dspin_tx2->value(),
                ui->dspin_ty2->value(), ui->dspin_tz2->value());
            if (ui->tabWidget->currentIndex() == 2) emit affine(trans_affine);
          });
  connect(ui->dspin_ay,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            trans_affine = pca::getTransformation(
                ui->dspin_angle->value(), ui->dspin_ax->value(), value,
                ui->dspin_az->value(), ui->dspin_tx2->value(),
                ui->dspin_ty2->value(), ui->dspin_tz2->value());
            if (ui->tabWidget->currentIndex() == 2) emit affine(trans_affine);
          });
  connect(ui->dspin_az,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            trans_affine = pca::getTransformation(
                ui->dspin_angle->value(), ui->dspin_ax->value(),
                ui->dspin_ay->value(), value, ui->dspin_tx2->value(),
                ui->dspin_ty2->value(), ui->dspin_tz2->value());
            if (ui->tabWidget->currentIndex() == 2) emit affine(trans_affine);
          });
  connect(ui->dspin_tx2,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            trans_affine = pca::getTransformation(
                ui->dspin_angle->value(), ui->dspin_ax->value(),
                ui->dspin_ay->value(), ui->dspin_az->value(), value,
                ui->dspin_ty2->value(), ui->dspin_tz2->value());
            if (ui->tabWidget->currentIndex() == 2) emit affine(trans_affine);
          });
  connect(ui->dspin_ty2,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            trans_affine = pca::getTransformation(
                ui->dspin_angle->value(), ui->dspin_ax->value(),
                ui->dspin_ay->value(), ui->dspin_az->value(),
                ui->dspin_tx2->value(), value, ui->dspin_tz2->value());
            if (ui->tabWidget->currentIndex() == 2) emit affine(trans_affine);
          });
  connect(ui->dspin_tz2,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            trans_affine = pca::getTransformation(
                ui->dspin_angle->value(), ui->dspin_ax->value(),
                ui->dspin_ay->value(), ui->dspin_az->value(),
                ui->dspin_tx2->value(), ui->dspin_ty2->value(), value);
            if (ui->tabWidget->currentIndex() == 2) emit affine(trans_affine);
          });
		  
  // singal
  connect(this, &Transformation::affine, [=](const Eigen::Affine3f &affine3f) {
    float x, y, z, rx, ry, rz;
    pca::getTranslationAndEulerAngles(affine3f, x, y, z, rx, ry, rz);
    float alpha, axisX, axisY, axisZ;
    pca::getAngleAxis(affine3f, alpha, axisX, axisY, axisZ);
    int index = ui->tabWidget->currentIndex();
    // matrix
    if (index != 0)
      ui->txt_matrix->setText(pca::getTransformationQString(affine3f, 3));
    // eulerAngle
    if (index != 1) {
      ui->dspin_rx1->setValue(rx);
      ui->dspin_ry1->setValue(ry);
      ui->dspin_rz1->setValue(rz);
      ui->dspin_tx1->setValue(x);
      ui->dspin_ty1->setValue(y);
      ui->dspin_tz1->setValue(z);
    }
    // axisAngle
    if (index != 2) {
      ui->dspin_angle->setValue(alpha);
      ui->dspin_ax->setValue(axisX);
      ui->dspin_ay->setValue(axisY);
      ui->dspin_az->setValue(axisZ);
      ui->dspin_tx2->setValue(x);
      ui->dspin_ty2->setValue(y);
      ui->dspin_tz2->setValue(z);
    }
  });
}

Transformation::~Transformation() { delete ui; }

void Transformation::init() {
  connect(this, &Transformation::affine, this, &Transformation::preview);
}

void Transformation::add() {
  if (!checkValid()) return;
  for (auto &cloud : selected_clouds) {
    cloudview->removePointCloud(cloud->id() + "_affine");
    pca::Cloud::Ptr new_cloud(new pca::Cloud(*cloud));
    if (!ui->cbox_inverse->isChecked())
      pcl::transformPointCloud(*cloud, *new_cloud,
                               trans_map.find(cloud->id())->second);
    else
      pcl::transformPointCloud(*cloud, *new_cloud,
                               trans_map.find(cloud->id())->second.inverse());
    new_cloud->setId("trans-" + cloud->id());
    new_cloud->update();
    cloudtree->appendCloud(cloud, new_cloud, true);
    trans_map.erase(cloud->id());
  }
  log(pca::LOG_INFO, "the transformed clouds added done!");
  cloudview->clearInfo();
}

void Transformation::apply() {
  if (!checkValid()) return;
  for (auto &cloud : selected_clouds) {
    cloudview->removePointCloud(cloud->id() + "_affine");
    if (!ui->cbox_inverse->isChecked())
      pcl::transformPointCloud(*cloud, *cloud,
                               trans_map.find(cloud->id())->second);
    else
      pcl::transformPointCloud(*cloud, *cloud,
                               trans_map.find(cloud->id())->second.inverse());
    cloud->update();
    cloudview->addPointCloud(cloud);
    cloudview->addBox(cloud);
    trans_map.erase(cloud->id());
  }
  log(pca::LOG_INFO, "the transformed clouds applied done!");
  cloudview->clearInfo();
}

void Transformation::reset() {
  for (auto &cloud : trans_map)
    cloudview->removePointCloud(cloud.first + "_affine");
  trans_map.clear();
  cloudview->clearInfo();
}

void Transformation::preview(const Eigen::Affine3f &affine3f) {
  trans_affine = affine3f;
  if (!ui->check_refresh->isChecked()) return;
  if (!checkValid(true)) return;
  cloudview->showInfo("Transform Pointcloud", 1);
  for (auto &cloud : selected_clouds) {
    pca::Cloud::Ptr trans_cloud = cloud->makeShared();
    trans_cloud->setId(cloud->id() + "_affine");
    cloudview->addPointCloud(trans_cloud);
    if (!ui->cbox_inverse->isChecked()) {
      cloudview->updateCloudPose(trans_cloud->id(), trans_affine);
      trans_map[cloud->id()] = trans_affine;
    } else {
      cloudview->updateCloudPose(trans_cloud->id(), trans_affine.inverse());
      trans_map[cloud->id()] = trans_affine.inverse();
    }
    cloudview->setPointCloudSize(trans_cloud->id(), 3);
  }
}

bool Transformation::checkValid(bool preview) {
  selected_clouds = cloudtree->getSelectedClouds();
  if (selected_clouds.empty()) {
    log(pca::LOG_WARNING, "please select a pointcloud!");
    return false;
  }
  if (trans_affine.matrix() == Eigen::Matrix4f::Identity()) {
    log(pca::LOG_WARNING, "Please transform the pointcloud first!");
    return false;
  }
  if (!preview)
    for (auto &cloud : selected_clouds)
      if (trans_map.find(cloud->id()) == trans_map.end())
        this->preview(trans_affine);
  return true;
}

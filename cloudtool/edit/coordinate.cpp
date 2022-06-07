#include "coordinate.h"

#include "pca/common.h"
#include "ui_coordinate.h"

Coordinate::Coordinate(QWidget *parent)
    : CustomDialog(parent), ui(new Ui::Coordinate) {
  ui->setupUi(this);
  connect(ui->btn_add, &QPushButton::clicked, this, &Coordinate::add);
  connect(ui->btn_reset, &QPushButton::clicked, this, &Coordinate::reset);
  connect(ui->btn_close, &QPushButton::clicked, this, &Coordinate::close);
  connect(ui->btn_add_coord, &QPushButton::clicked, this,
          &Coordinate::addCoord);
  connect(ui->btn_close_coord, &QPushButton::clicked, this,
          &Coordinate::closeCoord);
  connect(ui->btn_expand, &QPushButton::clicked, [=] {
    if (ui->widget->isHidden()) {
      ui->widget->show();
      ui->btn_expand->setIcon(QIcon(":/res/icon/collapse-text-input.svg"));
      this->setFixedHeight(155);
    } else {
      ui->widget->hide();
      ui->btn_expand->setIcon(QIcon(":/res/icon/expand-text-input.svg"));
      this->setFixedHeight(38);
    }
  });
  ui->widget->hide();
  this->setFixedHeight(38);
}

Coordinate::~Coordinate() { delete ui; }

void Coordinate::init() {
  connect(ui->dspin_scale,
          static_cast<void (QDoubleSpinBox::*)(double)>(
              &QDoubleSpinBox::valueChanged),
          [=](double value) {
            for (auto &coord : coordinate_vec) {
              coord.scale = value;
              cloudview->addCoordinateSystem(coord);
            }
          });

  pca::Coord default_coord(1.0);
  cloudview->addCoordinateSystem(default_coord);
  coordinate_vec.push_back(default_coord);
}

void Coordinate::add() {
  std::vector<pca::Cloud::Ptr> selectedClouds = cloudtree->getSelectedClouds();
  if (selectedClouds.empty()) return;
  for (auto &i : selectedClouds)
    if (!cloudview->contains(i->id() + "_coord")) {
      pca::Coord cloud_coord(i->id() + "_coord", ui->dspin_scale->value(),
                             i->box().pose);
      cloudview->addCoordinateSystem(cloud_coord);
      coordinate_vec.push_back(cloud_coord);
    }
}

void Coordinate::reset() {
  coordinate_vec.clear();
  pca::Coord default_coord(1.0);
  cloudview->addCoordinateSystem(default_coord);
  coordinate_vec.push_back(default_coord);
  ui->dspin_scale->setValue(0);
  cloudview->removeAllCoordinateSystems();
}

void Coordinate::addCoord() {
  if (ui->lineEdit_coord_id->text().isEmpty()) {
    log(pca::LOG_WARNING, "please input a id!");
    return;
  }
  QString id = ui->lineEdit_coord_id->text();
  if (cloudview->contains(id)) {
    log(pca::LOG_WARNING,
        "the id " + id +
            " already exists! Please rename a different id and retry.");
    return;
  }
  Eigen::Affine3f affine;
  if (!pca::getTransformation(ui->txt_matrix->toPlainText(), affine)) {
    log(pca::LOG_WARNING, "the transformation matrix format is wrong");
    return;
  }
  pca::Coord added_coord(id, ui->dspin_scale->value(), affine);
  cloudview->addCoordinateSystem(added_coord);
  coordinate_vec.push_back(added_coord);
}

void Coordinate::closeCoord() {
  QString id = ui->lineEdit_coord_id->text();
  if (!cloudview->contains(id)) {
    log(pca::LOG_WARNING, "the id " + id + " dose not exist! ");
    return;
  }
  cloudview->removeCoordinateSystem(id);
}

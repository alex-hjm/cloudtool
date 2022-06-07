#include "color.h"

#include "pca/common.h"
#include "ui_color.h"

Color::Color(QWidget *parent)
    : CustomDock(parent),
      ui(new Ui::Color),
      field_name(""),
      red(-1),
      green(-1),
      blue(-1) {
  ui->setupUi(this);
  ui->gridLayout->setSpacing(0);
  for (int row = 0; row < 5; row++) {
    for (int column = 0; column < 10; column++) {
      QPushButton *btn = new QPushButton();
      btn->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
      btn->setFixedHeight(20);
      if (row == 4 && column == 9) {
        btn->setText("+");
        btn->setStyleSheet(
            tr("QPushButton{border:none;border-radius:4px;background-color:"
               "transparent;}"
               "QPushButton:pressed{background-color:lightgray;}"));
        connect(btn, &QPushButton::clicked, [=] {
          QColor color =
              QColorDialog::getColor(Qt::white, this, tr("select color"));
          emit rgb(color.red(), color.green(), color.blue());
        });
      } else {
        btn->setStyleSheet(
            tr("QPushButton{border:none;border-radius:4px;background-color:rgb("
               "%1, %2, %3);}"
               "QPushButton:pressed{background-color:lightgray;}")
                .arg((colors[row][column]).red())
                .arg((colors[row][column]).green())
                .arg((colors[row][column]).blue()));
        connect(btn, &QPushButton::clicked, [=] {
          QColor color = colors[row][column];
          emit rgb(color.red(), color.green(), color.blue());
        });
      }
      ui->gridLayout->addWidget(btn, row, column);
    }
  }
  connect(ui->btn_x, &QPushButton::clicked, [=] { emit fieldName("x"); });
  connect(ui->btn_y, &QPushButton::clicked, [=] { emit fieldName("y"); });
  connect(ui->btn_z, &QPushButton::clicked, [=] { emit fieldName("z"); });
  connect(ui->btn_apply, &QPushButton::clicked, this, &Color::apply);
  connect(ui->btn_reset, &QPushButton::clicked, this, &Color::reset);
}

Color::~Color() { delete ui; }

void Color::init() {
  connect(this, &Color::rgb, [=](int r, int g, int b) {
    red = r;
    green = g;
    blue = b;
    field_name = "";
    switch (ui->cbox_type->currentIndex()) {
      case 0:  // pointcloud
        if (!checkValid()) return;
        for (auto &cloud : selected_clouds)
          cloudview->setPointCloudColor(cloud, r, g, b);
        break;
      case 1:  // background
        cloudview->setBackgroundColor(r, g, b);
        break;
      case 2:  // pointcloud normals
        if (!checkValid()) return;
        for (auto &cloud : selected_clouds)
          if (cloudview->contains(cloud->normalId()))
            cloudview->setPointCloudColor(cloud->normalId(), r, g, b);
        break;
      case 3:  // box
        if (!checkValid()) return;
        for (auto &cloud : selected_clouds)
          if (cloudview->contains(cloud->boxId()))
            cloudview->setShapeColor(cloud->boxId(), r, g, b);
        break;
    }
  });

  connect(this, &Color::fieldName, [=](const QString &name) {
    field_name = name;
    red = green = blue = -1;
    if (!checkValid()) return;
    for (auto &i : selected_clouds) cloudview->setPointCloudColor(i, name);
  });
}

void Color::apply() {
  if ((red == -1) && (field_name == "")) return;
  if (ui->cbox_type->currentIndex() == 1) return;
  if (!checkValid()) return;
  switch (ui->cbox_type->currentIndex()) {
    case 0:  // pointcloud
      for (auto &cloud : selected_clouds) {
        if (red != -1) {
          cloud->setCloudColor(red, green, blue);
          log(pca::LOG_INFO, "the pointcloud [id: " + cloud->id() +
                                 tr("] color has changed to rgb(%1,%2,%3).")
                                     .arg(red)
                                     .arg(green)
                                     .arg(blue));
        } else {
          cloud->setCloudColor(field_name);
          log(pca::LOG_INFO, "the pointcloud [id: " + cloud->id() +
                                 tr("] color has changed along the axis ") +
                                 field_name);
        }
        cloudview->addPointCloud(cloud);
      }
      break;
    case 2:  // pointcloudnormals
      for (auto &cloud : selected_clouds) {
        cloud->setNormalColor(red, green, blue);
        log(pca::LOG_INFO, "the pointcloud normals [id: " + cloud->normalId() +
                               tr("] color has changed to rgb(%1,%2,%3).")
                                   .arg(red)
                                   .arg(green)
                                   .arg(blue));
      }
      break;
    case 3:  // box
      for (auto &cloud : selected_clouds) {
        cloud->setBoxColor(red, green, blue);
        log(pca::LOG_INFO,
            "the pointcloud bounding box [id: " + cloud->boxId() +
                tr("] color has changed to rgb(%1,%2,%3).")
                    .arg(red)
                    .arg(green)
                    .arg(blue));
      }
      break;
  }
}

void Color::reset() {
  red = green = blue = -1;
  field_name = "";
  switch (ui->cbox_type->currentIndex()) {
    case 0:  // pointcloud
      for (auto &cloud : selected_clouds)
        cloudview->resetPointCloudColor(cloud);
      break;
    case 1:  // background
      cloudview->resetBackgroundColor();
      break;
    case 2:  // pointcloudnormals
      for (auto &cloud : selected_clouds)
        if (cloudview->contains(cloud->normalId()))
          cloudview->setPointCloudColor(cloud->normalId(), 255, 255, 255);
      break;
    case 3:  // box
      for (auto &cloud : selected_clouds)
        if (cloudview->contains(cloud->boxId()))
          cloudview->setShapeColor(cloud->boxId(), 255, 255, 255);
      break;
  }
}

bool Color::checkValid() {
  selected_clouds = cloudtree->getSelectedClouds();
  if (selected_clouds.empty()) {
    log(pca::LOG_WARNING, "please select a pointcloud!");
    return false;
  }
  return true;
}

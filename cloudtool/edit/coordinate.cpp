/**
 * @file coordinate.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-07
 */
#include "coordinate.h"
#include "ui_coordinate.h"

#include "common/resource.h"
#define COORD_DEFAULT_ID    "coordinate"

enum CoordinateType {
    Origin = 0,
    PointCloud,
    Manual 
};

enum CoordinatePose {
    EulerAngle = 0,
    Matrix
};

Coordinate::Coordinate(QWidget* parent): CustomDock(parent), 
    ui(new Ui::Coordinate)
{
    ui->setupUi(this);
    ui->tabWidget->setCurrentIndex(EulerAngle);
    ui->cbox_type->setCurrentIndex(Origin);
    ui->dspin_scale->setSingleStep(ui->dspin_step->value());
    connect(ui->btn_add, &QPushButton::clicked, this, &Coordinate::add);
    connect(ui->btn_remove, &QPushButton::clicked, this, &Coordinate::remove);
    connect(ui->btn_clear, &QPushButton::clicked, this, &Coordinate::clear);

    connect(ui->dspin_step, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value) {
            ui->dspin_scale->setSingleStep(value);
        });

    connect(ui->dspin_scale, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value) {
            if(-1 == ui->cbox_coordlist->currentIndex()) return;
            auto coord = ui->cbox_coordlist->currentData().value<ct::Coord>();
            coord.scale = value;
            ui->cbox_coordlist->setItemData(ui->cbox_coordlist->currentIndex(),QVariant::fromValue(coord));
            m_cloudview->addCoord(coord);
        });
}

Coordinate::~Coordinate() 
{ 
    delete ui; 
}

void Coordinate::add()
{
    auto getCoordId = [=]() -> QString {
        int count = ui->cbox_coordlist->count();
        if (count == 0) {
            return "coord_0";
        } else {
            return tr("coord_%1").arg(ui->cbox_coordlist->itemText(count - 1).split("_").back().toInt() + 1);
        }
    };

    std::vector<ct::Coord> coords{};
    switch (ui->cbox_type->currentIndex()) {
    case Origin: 
        coords.push_back(ct::Coord{COORD_DEFAULT_ID, 1, Eigen::Affine3f::Identity()});
        break;
    case PointCloud:
        for(auto cloud: m_cloudlist->getSelectedClouds()) {
            coords.push_back(ct::Coord{COORD_DEFAULT_ID, 1, cloud->bbox().pose});
        }
        break;
    case Manual:
        break;  
    }
    for(auto& coord : coords) {
        coord.scale = ui->dspin_scale->value();
        coord.id = getCoordId();
        ui->cbox_coordlist->addItem(QIcon(ICON_COORD), coord.id, QVariant::fromValue(coord));
        ui->cbox_coordlist->setCurrentText(coord.id);
        m_cloudview->addCoord(coord);
    }
}

void Coordinate::remove()
{
    QString coordId = ui->cbox_coordlist->currentText();
    if(m_cloudview->contains(coordId)) {
        m_cloudview->removeCoord(coordId);
        ui->cbox_coordlist->removeItem(ui->cbox_coordlist->currentIndex());
    }
}

void Coordinate::clear()
{
    ui->cbox_coordlist->clear();
    m_cloudview->removeAllCoords();
}

void Coordinate::handleLanguageChanged()
{
    ui->retranslateUi(this);
}
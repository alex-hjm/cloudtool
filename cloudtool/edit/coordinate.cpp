/**
 * @file coordinate.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-03-07
 */
#include "coordinate.h"
#include "ui_coordinate.h"

#include "common/resource.h"
#include "common/transforms.h"

#define COORD_DEFAULT_ID    "coordinate"
#define MANUAL_COORD_ID     "manual_coord"

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
    ui(new Ui::Coordinate), 
    m_pick_flag(false)
{
    ui->setupUi(this);
    ui->tabWidget->setCurrentIndex(EulerAngle);
    ui->cbox_type->setCurrentIndex(Origin);
    ui->dspin_scale->setSingleStep(ui->dspin_step->value());
    connect(ui->btn_add, &QPushButton::clicked, this, &Coordinate::add);
    connect(ui->btn_remove, &QPushButton::clicked, this, &Coordinate::remove);
    connect(ui->btn_clear, &QPushButton::clicked, this, &Coordinate::clear);

    connect(ui->dspin_step, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double value) {
            ui->dspin_scale->setSingleStep(value);
            ui->dspin_rx->setSingleStep(value);
            ui->dspin_ry->setSingleStep(value);
            ui->dspin_rz->setSingleStep(value);
            ui->dspin_tx->setSingleStep(value);
            ui->dspin_ty->setSingleStep(value);
            ui->dspin_tz->setSingleStep(value);
        });

    connect(ui->dspin_scale, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [=](double value) {
            if(-1 == ui->cbox_coordlist->currentIndex()) return;
            auto coord = ui->cbox_coordlist->currentData().value<ct::Coord>();
            coord.scale = value;
            ui->cbox_coordlist->setItemData(ui->cbox_coordlist->currentIndex(),QVariant::fromValue(coord));
            m_cloudview->addCoord(coord);
        });

    connect(ui->cbox_type, QOverload<int>::of(&QComboBox::currentIndexChanged), [=](int idx) {
                if(idx != Manual) {
                    ui->btn_add->setIcon(QIcon(ICON_ADD));
                    disconnect(m_cloudview, &ct::CloudView::pointPickEvent, this, &Coordinate::handlePreivewPickCoord);
                    m_pick_flag =false;
                    m_cloudview->removeCoord(MANUAL_COORD_ID);
                    m_coord.id = "";
                } else {
                    ui->btn_add->setIcon(QIcon(ICON_START));
                }
            });

    connect(ui->txt_matrix, &QTextEdit::textChanged, this, QOverload<void>::of(&Coordinate::handleCoordPoseChanged));
    connect(ui->dspin_rx, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, QOverload<double>::of(&Coordinate::handleCoordPoseChanged));  
    connect(ui->dspin_ry, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, QOverload<double>::of(&Coordinate::handleCoordPoseChanged));  
    connect(ui->dspin_rz, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, QOverload<double>::of(&Coordinate::handleCoordPoseChanged));  
    connect(ui->dspin_tx, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, QOverload<double>::of(&Coordinate::handleCoordPoseChanged));  
    connect(ui->dspin_ty, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, QOverload<double>::of(&Coordinate::handleCoordPoseChanged));  
    connect(ui->dspin_tz, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, QOverload<double>::of(&Coordinate::handleCoordPoseChanged));  
    connect(ui->cbox_coordlist, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &Coordinate::handleCurrentCoordPose);  

    
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
        coords.emplace_back(std::move(ct::Coord{COORD_DEFAULT_ID, 1, Eigen::Affine3f::Identity()}));
        break;
    case PointCloud:
        for(auto cloud: m_cloudlist->getSelectedClouds()) {
            coords.emplace_back(std::move(ct::Coord{COORD_DEFAULT_ID, 1, cloud->bbox().pose}));
        }
        break;
    case Manual:
        if(!m_pick_flag) {
            ui->btn_add->setIcon(QIcon(ICON_STOP));
            connect(m_cloudview, &ct::CloudView::pointPickEvent, this, &Coordinate::handlePreivewPickCoord);
            m_pick_flag =true;
        } else {
            ui->btn_add->setIcon(QIcon(ICON_START));
            disconnect(m_cloudview, &ct::CloudView::pointPickEvent, this, &Coordinate::handlePreivewPickCoord);
            m_cloudview->removeCoord(MANUAL_COORD_ID);
            if(!m_coord.id.isEmpty()) coords.emplace_back(m_coord);
            m_pick_flag =false;
            m_coord.id = "";
        }
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
    ui->dspin_tx->setValue(0);
    ui->dspin_ty->setValue(0);
    ui->dspin_tz->setValue(0);
    ui->dspin_rx->setValue(0);
    ui->dspin_ry->setValue(0);
    ui->dspin_rz->setValue(0);
}

void Coordinate::handleLanguageChanged()
{
    ui->retranslateUi(this);
}

void Coordinate::handleCoordPoseChanged()
{
    if(ui->tabWidget->currentIndex() != Matrix) return;
    bool success{false};
    auto affine = getTransformation(ui->txt_matrix->toPlainText(), success);
    if (success) {
        float x, y, z, rx, ry, rz;
        getTranslationAndEulerAngles(affine, x, y, z, rx, ry, rz);
        ui->dspin_tx->setValue(x);
        ui->dspin_ty->setValue(y);
        ui->dspin_tz->setValue(z);
        ui->dspin_rx->setValue(rx);
        ui->dspin_ry->setValue(ry);
        ui->dspin_rz->setValue(rz);
        if(-1 == ui->cbox_coordlist->currentIndex()) return;
        auto coord = ui->cbox_coordlist->currentData().value<ct::Coord>();
        coord.pose = affine;
        ui->cbox_coordlist->setItemData(ui->cbox_coordlist->currentIndex(),QVariant::fromValue(coord));
        m_cloudview->addCoord(coord);
    }
}

void Coordinate::handleCoordPoseChanged(double)
{
    if(ui->tabWidget->currentIndex() != EulerAngle) return;
    auto affine = getTransformation(ui->dspin_tx->value(), ui->dspin_ty->value(), ui->dspin_tz->value(),
                                                ui->dspin_rx->value(), ui->dspin_ry->value(), ui->dspin_rz->value());
    ui->txt_matrix->setText(getTransformationQString(affine.matrix(), 3));
    if(-1 == ui->cbox_coordlist->currentIndex()) return;
    auto coord = ui->cbox_coordlist->currentData().value<ct::Coord>();
    coord.pose = affine;
    ui->cbox_coordlist->setItemData(ui->cbox_coordlist->currentIndex(),QVariant::fromValue(coord));
    m_cloudview->addCoord(coord);
}

void Coordinate::handleCurrentCoordPose(int idx)
{
    if(-1 == idx) return;
    auto coord = ui->cbox_coordlist->currentData().value<ct::Coord>();
    float x, y, z, rx, ry, rz;
    getTranslationAndEulerAngles(coord.pose, x, y, z, rx, ry, rz);
    ui->dspin_tx->setValue(x);
    ui->dspin_ty->setValue(y);
    ui->dspin_tz->setValue(z);
    ui->dspin_rx->setValue(rx);
    ui->dspin_ry->setValue(ry);
    ui->dspin_rz->setValue(rz);
}

void Coordinate::handlePreivewPickCoord(const ct::PointPickInfo& info)
{
    if (!info.id.isEmpty()) {
        m_coord.id = MANUAL_COORD_ID;
        m_coord.pose = getTransformation(info.x, info.y, info.z, 0, 0, 0);
        m_coord.scale = ui->dspin_scale->value();
        m_cloudview->addCoord(m_coord);
    }
}
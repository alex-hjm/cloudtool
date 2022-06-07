/**
 * @file boundingbox.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-12
 */
#include "boundingbox.h"

#include "modules/features.h"
#include "ui_boundingbox.h"

BoundingBox::BoundingBox(QWidget* parent)
    : CustomDock(parent), ui(new Ui::BoundingBox), box_type(1)
{
    ui->setupUi(this);
    connect(ui->btn_apply, &QPushButton::clicked, this, &BoundingBox::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &BoundingBox::reset);
    connect(ui->btn_preview, &QPushButton::clicked, this, &BoundingBox::preview);
    connect(ui->check_points, &QCheckBox::clicked, [=](bool state)
            {
                if (state)
                {
                    box_type = 0;ui->check_wireframe->setChecked(false);
                    ui->check_surface->setChecked(false);
                }
            });
    connect(ui->check_wireframe, &QCheckBox::clicked, [=](bool state)
            {
                if (state)
                {
                    box_type = 1;
                    ui->check_points->setChecked(false);
                    ui->check_surface->setChecked(false);
                } 
            });
    connect(ui->check_surface, &QCheckBox::clicked, [=](bool state)
            {
                if (state)
                {
                    box_type = 2;
                    ui->check_points->setChecked(false);
                    ui->check_wireframe->setChecked(false);
                } 
            });
    connect(ui->dspin_rx, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            { emit eulerAngles(value, ui->dspin_ry->value(), ui->dspin_rz->value()); });
    connect(ui->dspin_ry, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            [=](double value)
            {
                emit eulerAngles(ui->dspin_rx->value(), value,
                                 ui->dspin_rz->value());
            });
    connect(ui->dspin_rz,
            static_cast<void (QDoubleSpinBox::*)(double)>(
                &QDoubleSpinBox::valueChanged),
            [=](double value)
            {
                emit eulerAngles(ui->dspin_rx->value(), ui->dspin_ry->value(),
                                 value);
            });
}

BoundingBox::~BoundingBox() { delete ui; }

void BoundingBox::preview()
{
    std::vector<pca::Cloud::Ptr> selectedClouds = cloudtree->getSelectedClouds();
    if (selectedClouds.empty())
    {
        log(pca::LOG_WARNING, "please select a pointcloud!");
        return;
    }
    this->adjustEnable(false);
    for (auto& i : selectedClouds)
    {
        pca::Box box;
        if (ui->rbtn_aabb->isChecked())
        {
            box = pca::Features::boundingBoxAABB(i);
            cloudview->addCube(box, i->boxId());
            cloudview->setShapeColor(i->boxId(), 255, 0, 0);
            cloudview->showInfo("Axis-Aligned Bounding Box", 1);
        }
        else
        {
            box = pca::Features::boundingBoxOBB(i);
            cloudview->addCube(box, i->boxId());
            cloudview->setShapeColor(i->boxId(), 0, 255, 0);
            cloudview->showInfo("Oriented Bounding Box", 1);
        }
        cloudview->setShapeRepersentation(i->boxId(), box_type);
        if (box_type == 0)
            cloudview->setShapeSize(i->boxId(), 5);
        else if (box_type == 1)
            cloudview->setShapeLineWidth(i->boxId(), 3);
        float rx, ry, rz;
        pca::getEulerAngles(box.pose, rx, ry, rz);
        ui->dspin_rx->setValue(rx);
        ui->dspin_ry->setValue(ry);
        ui->dspin_rz->setValue(rz);
        boxs_map[i->id()] = box;
    }
    this->adjustEnable(true);
}

void BoundingBox::apply()
{
    std::vector<pca::Cloud::Ptr> selectedClouds = cloudtree->getSelectedClouds();
    if (selectedClouds.empty())
    {
        log(pca::LOG_WARNING, "please select a pointcloud!");
        return;
    }
    for (auto& cloud : selectedClouds)
        if (boxs_map.find(cloud->id()) == boxs_map.end())
        {
            this->preview();
            break;
        }
    cloudview->clearInfo();
    for (auto& cloud : selectedClouds)
    {
        cloud->setCloudBox(boxs_map.find(cloud->id())->second);
        cloudview->addBox(cloud);
    }
    log(pca::LOG_INFO, "the bounding boxs has applied successfully!");
    this->adjustEnable(false);
}

void BoundingBox::reset()
{
    boxs_map.clear();
    cloudview->clearInfo();
    for (auto& cloud : cloudtree->getSelectedClouds())
    {
        cloudview->addBox(cloud);
    }
    this->adjustEnable(false);
}

void BoundingBox::adjustEnable(bool state)
{
    if (state)
    {
        connect(this, &BoundingBox::eulerAngles, this, &BoundingBox::adjustBox);
        ui->dspin_rx->setEnabled(true);
        ui->dspin_ry->setEnabled(true);
        ui->dspin_rz->setEnabled(true);
    }
    else
    {
        disconnect(this, &BoundingBox::eulerAngles, this, &BoundingBox::adjustBox);
        ui->dspin_rx->setEnabled(false);
        ui->dspin_ry->setEnabled(false);
        ui->dspin_rz->setEnabled(false);
    }
}

void BoundingBox::adjustBox(float r, float p, float y)
{
    for (auto& cloud : cloudtree->getSelectedClouds())
    {
        Eigen::Affine3f affine = pca::getTransformation(
            cloud->center()[0], cloud->center()[1], cloud->center()[2], r, p, y);
        pca::Box box = pca::Features::boundingBoxAdjust(cloud, affine.inverse());
        cloudview->addCube(box, cloud->boxId());
        cloudview->setShapeColor(cloud->boxId(), 0, 0, 255);
        cloudview->setShapeRepersentation(cloud->boxId(), box_type);
        if (box_type == 0)
            cloudview->setShapeSize(cloud->boxId(), 5);
        else if (box_type == 1)
            cloudview->setShapeLineWidth(cloud->boxId(), 3);
        boxs_map[cloud->id()] = box;
    }
}

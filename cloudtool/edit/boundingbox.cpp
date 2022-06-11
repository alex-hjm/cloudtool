/**
 * @file boundingbox.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-12
 */
#include "boundingbox.h"

#include "base/common.h"
#include "modules/features.h"

#include "ui_boundingbox.h"

#define BOX_TYPE_POINTS     (0)
#define BOX_TYPE_WIREFRAME  (1)
#define BOX_TYPE_SURFACE    (2)

#define CT_BoundingBox_AABB "Axis-Aligned Bounding Box"
#define CT_BoundingBox_OBB  "Oriented Bounding Box"

BoundingBox::BoundingBox(QWidget* parent)
    : CustomDock(parent), ui(new Ui::BoundingBox), m_box_type(BOX_TYPE_WIREFRAME)
{
    ui->setupUi(this);
    connect(ui->btn_apply, &QPushButton::clicked, this, &BoundingBox::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &BoundingBox::reset);
    connect(ui->btn_preview, &QPushButton::clicked, this, &BoundingBox::preview);
    connect(ui->check_points, &QCheckBox::clicked, [=](bool checked)
            {
                if (checked)
                {
                    m_box_type = BOX_TYPE_POINTS;
                    ui->check_wireframe->setChecked(false);
                    ui->check_surface->setChecked(false);
                }
            });
    connect(ui->check_wireframe, &QCheckBox::clicked, [=](bool checked)
            {
                if (checked)
                {
                    m_box_type = BOX_TYPE_WIREFRAME;
                    ui->check_points->setChecked(false);
                    ui->check_surface->setChecked(false);
                }
            });
    connect(ui->check_surface, &QCheckBox::clicked, [=](bool checked)
            {
                if (checked)
                {
                    m_box_type = BOX_TYPE_SURFACE;
                    ui->check_points->setChecked(false);
                    ui->check_wireframe->setChecked(false);
                }
            });
    connect(ui->dspin_rx, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            { emit eulerAngles(value, ui->dspin_ry->value(), ui->dspin_rz->value()); });
    connect(ui->dspin_ry, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            { emit eulerAngles(ui->dspin_rx->value(), value, ui->dspin_rz->value()); });
    connect(ui->dspin_rz, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            { emit eulerAngles(ui->dspin_rx->value(), ui->dspin_ry->value(), value); });
}

BoundingBox::~BoundingBox() { delete ui; }

void BoundingBox::preview()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        m_console->print(ct::LOG_WARNING, LOG_STATU_NO_POINTCLOUD);
        return;
    }
    this->adjustEnable(false);
    for (auto& i : selected_clouds)
    {
        ct::Box box;
        if (ui->rbtn_aabb->isChecked())
        {
            box = ct::Features::boundingBoxAABB(i);
            m_cloudview->addCube(box, i->boxId());
            m_cloudview->setShapeColor(i->boxId(), QColorConstants::Red);
            m_cloudview->showInfo(CT_BoundingBox_AABB, 1);
        }
        else
        {
            box = ct::Features::boundingBoxOBB(i);
            m_cloudview->addCube(box, i->boxId());
            m_cloudview->setShapeColor(i->boxId(), QColorConstants::Green);
            m_cloudview->showInfo(CT_BoundingBox_OBB, 1);
        }
        m_cloudview->setShapeRepersentation(i->boxId(), m_box_type);
        switch (m_box_type)
        {
        case BOX_TYPE_POINTS:
            m_cloudview->setShapeSize(i->boxId(), 5);
            break;
        case BOX_TYPE_WIREFRAME:
            m_cloudview->setShapeLineWidth(i->boxId(), 3);
            break;
        case BOX_TYPE_SURFACE:
            m_cloudview->setShapeOpacity(i->boxId(), 0.5);
            break;
        }
        float roll, pitch, yaw;
        pcl::getEulerAngles(box.pose, roll, pitch, yaw);
        ui->dspin_rx->setValue(pcl::rad2deg(roll));
        ui->dspin_ry->setValue(pcl::rad2deg(pitch));
        ui->dspin_rz->setValue(pcl::rad2deg(yaw));
        m_box_map[i->id()] = box;
    }
    this->adjustEnable(true);
    m_console->print(ct::LOG_INFO, LOG_STATU_PREVIEW_DONE(CT_BoundingBox));
}

void BoundingBox::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        m_console->print(ct::LOG_WARNING, LOG_STATU_NO_POINTCLOUD);
        return;
    }
    for (auto& cloud : selected_clouds)
        if (m_box_map.find(cloud->id()) == m_box_map.end())
        {
            this->preview();
            break;
        }
    m_cloudview->clearInfo();
    for (auto& cloud : selected_clouds)
    {
        cloud->setBox(m_box_map.find(cloud->id())->second);
        m_cloudview->addBox(cloud);
    }
    this->adjustEnable(false);
    m_console->print(ct::LOG_INFO, LOG_STATU_APPLY_DONE(CT_BoundingBox));
}

void BoundingBox::reset()
{
    m_box_map.clear();
    m_cloudview->clearInfo();
    for (auto& cloud : m_cloudtree->getSelectedClouds())
    {
        m_cloudview->addBox(cloud);
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
    for (auto& cloud : m_cloudtree->getSelectedClouds())
    {
        Eigen::Affine3f affine = pcl::getTransformation(cloud->center()[0], cloud->center()[1], cloud->center()[2],
                                                        pcl::deg2rad(r), pcl::deg2rad(p), pcl::deg2rad(y));
        ct::Box box = ct::Features::boundingBoxAdjust(cloud, affine.inverse());
        m_cloudview->addCube(box, cloud->boxId());
        m_cloudview->setShapeColor(cloud->boxId(), QColorConstants::Blue);
        m_cloudview->setShapeRepersentation(cloud->boxId(), m_box_type);
        switch (m_box_type)
        {
        case BOX_TYPE_POINTS:
            m_cloudview->setShapeSize(cloud->boxId(), 5);
            break;
        case BOX_TYPE_WIREFRAME:
            m_cloudview->setShapeLineWidth(cloud->boxId(), 3);
            break;
        case BOX_TYPE_SURFACE:
            m_cloudview->setShapeOpacity(cloud->boxId(), 0.5);
            break;
        }
        m_box_map[cloud->id()] = box;
    }
}

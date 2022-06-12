/**
 * @file scale.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#include "scale.h"

#include "ui_scale.h"

#define SCALE_PRE_FLAG "-scale"
#define SCALE_ADD_FLAG "scaled-"

Scale::Scale(QWidget* parent) : CustomDialog(parent), ui(new Ui::Scale)
{
    ui->setupUi(this);
    connect(ui->btn_add, &QPushButton::clicked, this, &Scale::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Scale::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Scale::reset);
    connect(ui->btn_close, &QPushButton::clicked, this, &Scale::close);

    connect(ui->check_samevalue, &QCheckBox::stateChanged, [=](int state)
            {
                if (state)
                {
                    ui->dspin_y->setEnabled(false);
                    ui->dspin_z->setEnabled(false);
                }
                else
                {
                    ui->dspin_y->setEnabled(true);
                    ui->dspin_z->setEnabled(true);
                }
            });

    connect(ui->dspin_x, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->check_samevalue->isChecked())
                {
                    ui->dspin_y->setValue(value);
                    ui->dspin_z->setValue(value);
                    emit scale(value, value, value);
                }
                else
                    emit scale(value, ui->dspin_y->value(), ui->dspin_z->value());
            });

    connect(ui->dspin_y, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->check_samevalue->isChecked()) return;
                emit scale(ui->dspin_x->value(), value, ui->dspin_z->value());
            });

    connect(ui->dspin_z, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double value)
            {
                if (ui->check_samevalue->isChecked()) return;
                emit scale(ui->dspin_x->value(), ui->dspin_y->value(), value);
            });
    ui->check_samevalue->setChecked(true);
    connect(this, &Scale::scale, this, &Scale::preview);
}

Scale::~Scale() { delete ui; }

void Scale::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_scale_map.find(cloud->id()) == m_scale_map.end())
        {
            printW(QString("The cloud[id:%1] has no estimated normals !").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_scale_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(new_cloud->id());
        new_cloud->setId(SCALE_ADD_FLAG + cloud->id());
        m_cloudtree->appendCloud(cloud, new_cloud, true);
        m_scale_map.erase(cloud->id());
        printI(QString("Add scaled cloud[id:%1] done.").arg(new_cloud->id()));
    }
    m_cloudview->clearInfo();
}

void Scale::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_scale_map.find(cloud->id()) == m_scale_map.end())
        {
            printW(QString("The cloud[id:%1] has no estimated normals !").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_scale_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(new_cloud->id());
        m_cloudtree->updateCloud(cloud, new_cloud);
        m_scale_map.erase(cloud->id());
        m_cloudtree->setCloudChecked(cloud);
        printI(QString("Apply scaled cloud[id:%1] done.").arg(new_cloud->id()));
    }
    m_cloudview->clearInfo();
}

void Scale::reset()
{
    for (auto& cloud : m_cloudtree->getSelectedClouds())
        m_cloudtree->setCloudChecked(cloud);
    for (auto& cloud : m_scale_map)
        m_cloudview->removePointCloud(cloud.second->id());
    m_scale_map.clear();
    m_cloudview->clearInfo();
}

void Scale::preview(double x, double y, double z)
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    m_cloudview->showInfo("Scale Pointcloud", 1);
    for (auto& cloud : selected_clouds)
    {
        m_cloudtree->setCloudChecked(cloud, false);
        ct::Cloud::Ptr scaled_cloud = cloud->makeShared();
        if (ui->cbox_type->currentIndex() == 0)
            scaled_cloud->scale(x, y, z, false);
        else if (ui->cbox_type->currentIndex() == 1)
            scaled_cloud->scale(x, y, z, true);
        if (ui->check_keepentity->isChecked()) m_cloudview->resetCamera();
        scaled_cloud->setId(scaled_cloud->id() + SCALE_PRE_FLAG);
        m_cloudview->addPointCloud(scaled_cloud);
        m_scale_map[cloud->id()] = scaled_cloud;
    }
}


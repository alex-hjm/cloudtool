/**
 * @file normals.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#include "normals.h"

#include "ui_normals.h"

#define NORMALS_ADD_FLAG    "normals-"

Normals::Normals(QWidget* parent)
    : CustomDock(parent), ui(new Ui::Normals), m_thread(this)
{
    ui->setupUi(this);

    qRegisterMetaType<Eigen::Vector3f>("Eigen::Vector3f &");
    qRegisterMetaType<Eigen::Vector3f>("Eigen::Vector3f");

    connect(ui->btn_preview, &QPushButton::clicked, this, &Normals::preview);
    connect(ui->btn_add, &QPushButton::clicked, this, &Normals::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Normals::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Normals::reset);

    m_feature = new ct::Features;
    m_feature->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_feature, &QObject::deleteLater);
    connect(this, &Normals::normalEstimation, m_feature, &ct::Features::NormalEstimation);
    connect(m_feature, &ct::Features::normalsResult, this, &Normals::normalsResult);
    m_thread.start();

    connect(ui->check_max, &QCheckBox::clicked, [=](bool state)
            {
                if (state)
                {
                    ui->check_center->setChecked(false);
                    ui->check_origin->setChecked(false);
                }
            });
    connect(ui->check_center, &QCheckBox::clicked, [=](bool state)
            {
                if (state)
                {
                    ui->check_max->setChecked(false);
                    ui->check_origin->setChecked(false);
                }
            });
    connect(ui->check_origin, &QCheckBox::clicked, [=](bool state)
            {
                if (state)
                {
                    ui->check_max->setChecked(false);
                    ui->check_center->setChecked(false);
                }
            });

    connect(ui->spin_level, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), [=](int)
            {
                if (ui->check_refresh->isChecked()) this->updateNormals();
            });
    connect(ui->dspin_scale, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), [=](double)
            {
                if (ui->check_refresh->isChecked()) this->updateNormals();
            });

    connect(ui->rbtn_k, &QRadioButton::clicked, [=](bool checked)
            {
                if (checked)
                {
                    ui->spin_k->setEnabled(true);
                    ui->dspin_r->setValue(0);
                    ui->dspin_r->setEnabled(false);
                }
            });
    connect(ui->rbtn_r, &QRadioButton::clicked, [=](bool checked)
            {
                if (checked)
                {
                    ui->dspin_r->setEnabled(true);
                    ui->spin_k->setValue(0);
                    ui->spin_k->setEnabled(false);
                }
            });
    connect(ui->check_reverse, &QCheckBox::stateChanged, this, &Normals::reverseNormals);
}

Normals::~Normals()
{
    m_thread.quit();
    m_thread.wait();
    delete ui;
}

void Normals::preview()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    if (ui->spin_k->value() == 0 && ui->dspin_r->value() == 0)
    {
        printW("Parameters set error!");
        return;
    }
    if (ui->rbtn_k->isChecked())
        m_cloudview->showInfo("K-nearest neighbor search estimation", 1);
    else
        m_cloudview->showInfo("R-radius search estimation", 1);
    for (auto& cloud : selected_clouds)
    {
        m_feature->setInputCloud(cloud);
        m_feature->setKSearch(ui->spin_k->value());
        m_feature->setRadiusSearch(ui->dspin_r->value());
        Eigen::Vector3f viewpoint;
        if (ui->check_center->isChecked())
            viewpoint = cloud->center();
        else if (ui->check_max->isChecked())
            viewpoint << std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max();
        else
            viewpoint << cloud->sensor_origin_.coeff(0), cloud->sensor_origin_.coeff(1), cloud->sensor_origin_.coeff(2);
        emit normalEstimation(viewpoint[0], viewpoint[1], viewpoint[2]);
        m_cloudtree->showProgressBar();
    }
}

void Normals::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_normals_map.find(cloud->id()) == m_normals_map.end())
        {
            printW(QString("The cloud[id:%1] has no estimated normals !").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_normals_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(new_cloud->normalId());
        new_cloud->setId(NORMALS_ADD_FLAG + new_cloud->id());
        m_cloudtree->appendCloud(cloud, new_cloud, true);
        m_normals_map.erase(cloud->id());
        printI(QString("Add cloud[id:%1] with estimated normals done.").arg(cloud->id()));
    }
    m_cloudview->clearInfo();
}

void Normals::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_normals_map.find(cloud->id()) == m_normals_map.end())
        {
            printW(QString("The cloud[id:%1] has no estimated normals !").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_normals_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(new_cloud->normalId());
        m_cloudtree->updateCloud(cloud, new_cloud);
        m_normals_map.erase(cloud->id());
        printI(QString("Apply cloud[id:%1] estimated normals done.").arg(cloud->id()));
    }
    m_cloudview->clearInfo();
}

void Normals::reset()
{
    for (auto& cloud : m_normals_map)
        m_cloudview->removePointCloud(cloud.second->normalId());
    m_normals_map.clear();
    m_cloudview->clearInfo();
    m_cloudtree->closeProgressBar();
}

void Normals::reverseNormals()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        ct::Cloud::Ptr cloud_with_normals = m_normals_map.find(cloud->id())->second;
        for (auto& point : cloud_with_normals->points)
        {
            point.normal_x = -point.normal_x;
            point.normal_y = -point.normal_y;
            point.normal_z = -point.normal_z;
        }
        m_cloudview->addPointCloudNormals(cloud_with_normals, ui->spin_level->value(), ui->dspin_scale->value());
    }
}

void Normals::updateNormals()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        ct::Cloud::Ptr cloud_with_normals = m_normals_map.find(cloud->id())->second;
        m_cloudview->addPointCloudNormals(cloud_with_normals, ui->spin_level->value(), ui->dspin_scale->value());
    }
}

void Normals::normalsResult(const ct::Cloud::Ptr& cloud, float time)
{
    if (ui->check_reverse->isChecked())
    {
        for (auto& point : cloud->points)
        {
            point.normal_x = -point.normal_x;
            point.normal_y = -point.normal_y;
            point.normal_z = -point.normal_z;
        }
    }
    m_cloudview->addPointCloudNormals(cloud, ui->spin_level->value(), ui->dspin_scale->value());
    m_normals_map[cloud->id()] = cloud;
    printI(QString("Estimate cloud[id:%1] normals done, take time %2 ms.").arg(cloud->id()).arg(time));
    m_cloudtree->closeProgressBar();
}


/**
 * @file scale.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#include "boundary.h"

#include "ui_boundary.h"

#define BOUNDARY_PRE_FLAG "-boundary"
#define BOUNDARY_ADD_FLAG "boundary-"

Boundary::Boundary(QWidget* parent)
    :CustomDialog(parent), ui(new Ui::Boundary),
    m_thread(this)
{
    ui->setupUi(this);
    connect(ui->btn_preview, &QPushButton::clicked, this, &Boundary::preview);
    connect(ui->btn_add, &QPushButton::clicked, this, &Boundary::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Boundary::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Boundary::reset);
    connect(ui->btn_close, &QPushButton::clicked, this, &Boundary::close);

    m_feature = new ct::Features;
    m_feature->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_feature, &QObject::deleteLater);
    connect(this, &Boundary::BoundaryEstimation, m_feature, &ct::Features::BoundaryEstimation);
    connect(m_feature, &ct::Features::boundaryResult, this, &Boundary::boundaryResult);
    m_thread.start();
}

Boundary::~Boundary()
{
    m_thread.quit();
    m_thread.wait();
    delete ui;
}


void Boundary::preview()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (ui->spin_k->value() == 0 && ui->dspin_r->value() == 0)
        {
            printW("Parameters set error!");
            return;
        }
        if (!cloud->hasNormals())
        {
            printW("Please estimate normals first!");
            return;
        }
        m_feature->setInputCloud(cloud);
        m_feature->setKSearch(ui->spin_k->value());
        m_feature->setRadiusSearch(ui->dspin_r->value());
        m_cloudview->showInfo("BoundaryEstimation", 1);
        emit BoundaryEstimation(ui->dspin_angle->value());
        m_cloudtree->showProgressBar();
    }
}

void Boundary::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_boundary_map.find(cloud->id()) == m_boundary_map.end())
        {
            printW(QString("The cloud[id:%1] has no matched boundary !").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_boundary_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(new_cloud->id());
        new_cloud->setId(BOUNDARY_ADD_FLAG + cloud->id());
        m_cloudtree->appendCloud(cloud, new_cloud, true);
        m_boundary_map.erase(cloud->id());
        printI(QString("Add cloud boundary[id:%1] done.").arg(new_cloud->id()));
    }
    m_cloudview->clearInfo();
}

void Boundary::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_boundary_map.find(cloud->id()) == m_boundary_map.end())
        {
            printW(QString("The cloud[id:%1] has no matched keypoints !").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_boundary_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(new_cloud->id());
        m_cloudtree->updateCloud(cloud, new_cloud);
        m_boundary_map.erase(cloud->id());
        m_cloudtree->setCloudChecked(cloud);
        printI(QString("Apply cloud boundary[id:%1] done.").arg(new_cloud->id()));
    }
    m_cloudview->clearInfo();
}

void Boundary::reset()
{
    for (auto& cloud : m_cloudtree->getSelectedClouds())
        m_cloudtree->setCloudChecked(cloud);
    for (auto& cloud : m_boundary_map)
        m_cloudview->removePointCloud(cloud.second->id());
    m_boundary_map.clear();
    m_cloudview->clearInfo();
}

void Boundary::boundaryResult(const ct::Cloud::Ptr& cloud, float time)
{
    printI(QString("Estimate cloud[id:%1] boundary done, take time %2 ms.").arg(cloud->id()).arg(time));
    QString id = cloud->id();
    cloud->setId(id + BOUNDARY_PRE_FLAG);
    m_cloudview->addPointCloud(cloud);
    m_cloudview->setPointCloudColor(cloud->id(), QColorConstants::Green);
    m_cloudview->setPointCloudSize(cloud->id(), cloud->pointSize() + 2);
    m_boundary_map[id] = cloud;
    m_cloudtree->closeProgressBar();
}



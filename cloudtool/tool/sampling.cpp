/**
 * @file sampling.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#include "sampling.h"
#include "ui_sampling.h"

#define SAMPLING_TYPE_DownSampling              (0)
#define SAMPLING_TYPE_UniformSampling           (1)
#define SAMPLING_TYPE_RandomSampling            (2)
#define SAMPLING_TYPE_ReSampling                (3)
#define SAMPLING_TYPE_SamplingSurfaceNormal     (4)
#define SAMPLING_TYPE_NormalSpaceSampling       (5)

#define SAMPLING_PRE_FLAG                       "-sampling"
#define SAMPLING_ADD_FLAG                       "sampling-"

Sampling::Sampling(QWidget* parent)
    :CustomDialog(parent), ui(new Ui::Sampling),
    m_thread(this)
{
    ui->setupUi(this);
    connect(ui->btn_preview, &QPushButton::clicked, this, &Sampling::preview);
    connect(ui->btn_add, &QPushButton::clicked, this, &Sampling::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Sampling::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Sampling::reset);
    connect(ui->btn_close, &QPushButton::clicked, this, &Sampling::close);

    m_filters = new ct::Filters;
    m_filters->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_filters, &QObject::deleteLater);
    connect(this, &Sampling::DownSampling, m_filters, &ct::Filters::DownSampling);
    connect(this, &Sampling::UniformSampling, m_filters, &ct::Filters::UniformSampling);
    connect(this, &Sampling::RandomSampling, m_filters, &ct::Filters::RandomSampling);
    connect(this, &Sampling::ReSampling, m_filters, &ct::Filters::ReSampling);
    connect(this, &Sampling::SamplingSurfaceNormal, m_filters, &ct::Filters::SamplingSurfaceNormal);
    connect(this, &Sampling::NormalSpaceSampling, m_filters, &ct::Filters::NormalSpaceSampling);
    connect(m_filters, &ct::Filters::filterResult, this, &Sampling::samplingResult);
    m_thread.start();

    connect(ui->cbox_type, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [=](int index)
            {
                if (index == 0 || index == 1)
                    this->setFixedSize(168, 85);
                if (index == 2 || index == 3)
                    this->setFixedSize(168, 112);
                if (index == 4 || index == 5)
                    this->setFixedSize(168, 136);
            });
    ui->cbox_type->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
    this->setFixedSize(168, 85);
}

Sampling::~Sampling()
{
    m_thread.quit();
    m_thread.wait();
    delete ui;
}
void Sampling::preview()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        m_filters->setInputCloud(cloud);
        m_filters->setNegative(ui->check_reverse->isChecked());
        switch (ui->cbox_type->currentIndex())
        {
        case SAMPLING_TYPE_DownSampling:
            m_cloudview->showInfo("DownSampling(VoxelGrid)", 1);
            emit DownSampling(ui->dspin_r1->value());
            break;
        case SAMPLING_TYPE_UniformSampling:
            m_cloudview->showInfo("UniformSampling", 1);
            emit UniformSampling(ui->dspin_r2->value());
            break;
        case SAMPLING_TYPE_RandomSampling:
            m_cloudview->showInfo("RadiusOutlierRemoval", 1);
            emit RandomSampling(ui->spin_sample1->value(), ui->spin_seed1->value());
            break;
        case SAMPLING_TYPE_ReSampling:
            m_cloudview->showInfo("ReSampling(MovingLeastSquares)", 1);
            emit ReSampling(ui->dspin_r3->value(),ui->spin_order->value());
            break;
        case SAMPLING_TYPE_SamplingSurfaceNormal:
            m_cloudview->showInfo("SamplingSurfaceNormal", 1);
            emit SamplingSurfaceNormal(ui->spin_sample2->value(),ui->spin_seed2->value(),ui->dspin_ratio->value());
            break;
        case SAMPLING_TYPE_NormalSpaceSampling:
            m_cloudview->showInfo("NormalSpaceSampling", 1);
            emit NormalSpaceSampling(ui->spin_sample3->value(),ui->spin_seed3->value(),ui->spin_bin->value());
            break;
        }
        m_cloudtree->showProgressBar();
    }
}

void Sampling::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_sampling_map.find(cloud->id()) == m_sampling_map.end())
        {
            printW(QString("The cloud[id:%1] has no sampling cloud !").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_sampling_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(new_cloud->id());
        new_cloud->setId(SAMPLING_ADD_FLAG + cloud->id());
        m_cloudtree->appendCloud(cloud, new_cloud, true);
        m_sampling_map.erase(cloud->id());
        printI(QString("Add sampling cloud[id:%1] done.").arg(new_cloud->id()));
    }
    m_cloudview->clearInfo();
}

void Sampling::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_sampling_map.find(cloud->id()) == m_sampling_map.end())
        {
            printW(QString("The cloud[id:%1] has no sampling cloud !").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_sampling_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(new_cloud->id());
        m_cloudtree->updateCloud(cloud, new_cloud);
        m_sampling_map.erase(cloud->id());
        m_cloudtree->setCloudChecked(cloud);
        printI(QString("Apply sampling cloud[id:%1] done.").arg(new_cloud->id()));
    }
    m_cloudview->clearInfo();
}

void Sampling::reset()
{
    for (auto& cloud : m_cloudtree->getSelectedClouds())
        m_cloudtree->setCloudChecked(cloud);
    for (auto& cloud : m_sampling_map)
        m_cloudview->removePointCloud(cloud.second->id());
    m_sampling_map.clear();
    m_cloudview->clearInfo();
}

void Sampling::samplingResult(const ct::Cloud::Ptr& cloud, float time)
{
    printI(QString("Sampling cloud[id:%1] done, take time %2 ms.").arg(cloud->id()).arg(time));
    QString id = cloud->id();
    cloud->setId(id + SAMPLING_PRE_FLAG);
    m_cloudview->addPointCloud(cloud);
    m_cloudview->setPointCloudColor(cloud->id(), ct::Color::Green);
    m_cloudview->setPointCloudSize(cloud->id(), cloud->pointSize() + 2);
    m_sampling_map[id] = cloud;
    m_cloudtree->closeProgressBar();
}

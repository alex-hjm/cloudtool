#include "keypoints.h"

#include "ui_keypoints.h"

#define KEYPOINTS_TYPE_NarfKeypoint         (0)
#define KEYPOINTS_TYPE_HarrisKeypoint3D     (1)
#define KEYPOINTS_TYPE_ISSKeypoint3D        (2)
#define KEYPOINTS_TYPE_SIFTKeypoint         (3)
#define KEYPOINTS_TYPE_TrajkovicKeypoint3D  (4)

#define KEYPOINTS_PRE_FLAG                  "-keypoints"
#define KEYPOINTS_ADD_FLAG                  "keypoints-"

KeyPoints::KeyPoints(QWidget* parent)
    : CustomDock(parent), ui(new Ui::KeyPoints),
    m_thread(this), m_rangeimage(nullptr)
{
    ui->setupUi(this);

    qRegisterMetaType<ct::RangeImage::Ptr>("ct::RangeImage::Ptr &");
    qRegisterMetaType<ct::RangeImage::Ptr>("ct::RangeImage::Ptr");

    connect(ui->btn_add, &QPushButton::clicked, this, &KeyPoints::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &KeyPoints::apply);
    connect(ui->btn_preview, &QPushButton::clicked, this, &KeyPoints::preview);
    connect(ui->btn_reset, &QPushButton::clicked, this, &KeyPoints::reset);

    m_keypoints = new ct::Keypoints;
    m_keypoints->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_keypoints, &QObject::deleteLater);
    connect(this, &KeyPoints::NarfKeypoint, m_keypoints, &ct::Keypoints::NarfKeypoint);
    connect(this, &KeyPoints::HarrisKeypoint3D, m_keypoints, &ct::Keypoints::HarrisKeypoint3D);
    connect(this, &KeyPoints::ISSKeypoint3D, m_keypoints, &ct::Keypoints::ISSKeypoint3D);
    connect(this, &KeyPoints::SIFTKeypoint, m_keypoints, &ct::Keypoints::SIFTKeypoint);
    connect(this, &KeyPoints::TrajkovicKeypoint3D, m_keypoints, &ct::Keypoints::TrajkovicKeypoint3D);
    connect(m_keypoints, &ct::Keypoints::keypointsResult, this, &KeyPoints::keypointsResult);
    m_thread.start();

    ui->cbox_type->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
}

KeyPoints::~KeyPoints()
{
    m_thread.quit();
    m_thread.wait();
    delete ui;
}

void KeyPoints::preview()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        m_keypoints->setInputCloud(cloud);
        m_keypoints->setKSearch(ui->spin_k->value());
        m_keypoints->setRadiusSearch(ui->dspin_r->value());
        switch (ui->cbox_type->currentIndex())
        {
        case KEYPOINTS_TYPE_NarfKeypoint:
            if (m_rangeimage == nullptr)
            {
                printW("Please get a range image first!");
                return;
            }
            m_cloudview->showInfo("NarfKeypoint", 1);
            emit NarfKeypoint(m_rangeimage->getRangeImage(), ui->dspin_support_size->value());
            break;
        case KEYPOINTS_TYPE_HarrisKeypoint3D:
            m_cloudview->showInfo("HarrisKeypoint3D", 1);
            emit  HarrisKeypoint3D(ui->cbox_response_method->currentIndex(), ui->dspin_threshold->value(), ui->check_non_maxima->isChecked(),
                                   ui->check_do_refine->isChecked());
            break;
        case KEYPOINTS_TYPE_ISSKeypoint3D:
            m_cloudview->showInfo("ISSKeypoint3D", 1);
            emit ISSKeypoint3D(cloud->resolution(), ui->dspin_gamma_21->value(), ui->dspin_gamma_32->value(),
                               ui->spin_min_neighbors->value(), ui->dspin_angle->value());
            break;
        case KEYPOINTS_TYPE_SIFTKeypoint:
            m_cloudview->showInfo("SIFTKeypoint", 1);
            emit SIFTKeypoint(ui->dspin_min_scale->value(), ui->spin_nr_octaves->value(), ui->spin_nr_scales_per_octave->value(),
                              ui->dspin_min_contrast->value());
            break;
        case KEYPOINTS_TYPE_TrajkovicKeypoint3D:
            m_cloudview->showInfo("TrajkovicKeypoint3D", 1);
            emit TrajkovicKeypoint3D(ui->cbox_compute_method->currentIndex(), ui->spin_window_size->value(),
                                     ui->dspin_first_threshold->value(), ui->dspin_second_threshold->value());
            break;
        }
        m_cloudtree->showProgressBar();
    }
}

void KeyPoints::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_keypoints_map.find(cloud->id()) == m_keypoints_map.end())
        {
            printW(QString("The cloud[id:%1] has no matched keypoints !").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_keypoints_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(cloud->id() + KEYPOINTS_ADD_FLAG);
        new_cloud->setId(KEYPOINTS_ADD_FLAG + cloud->id());
        m_cloudtree->appendCloud(cloud, new_cloud, true);
        m_keypoints_map.erase(cloud->id());
        printI(QString("Add cloud keypoints[id:%1] done.").arg(new_cloud->id()));
    }
    m_cloudview->clearInfo();
}

void KeyPoints::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_keypoints_map.find(cloud->id()) == m_keypoints_map.end())
        {
            printW(QString("The cloud[id:%1] has no matched keypoints !").arg(cloud->id()));
            continue;
        }
        ct::Cloud::Ptr new_cloud = m_keypoints_map.find(cloud->id())->second;
        m_cloudview->removePointCloud(new_cloud->id());
        m_cloudtree->updateCloud(cloud, new_cloud);
        m_keypoints_map.erase(cloud->id());
        m_cloudtree->setCloudChecked(cloud);
        printI(QString("Apply cloud keypoints[id:%1] done.").arg(new_cloud->id()));
    }
    m_cloudview->clearInfo();
}

void KeyPoints::reset()
{
    for (auto& cloud : m_cloudtree->getSelectedClouds())
        m_cloudtree->setCloudChecked(cloud);
    for (auto& cloud : m_keypoints_map)
        m_cloudview->removePointCloud(cloud.second->id());
    m_keypoints_map.clear();
    m_cloudview->clearInfo();
}

void KeyPoints::keypointsResult(const ct::Cloud::Ptr& cloud, float time)
{
    printI(QString("keypoints cloud[id:%1] done, take time %2 ms.").arg(cloud->id()).arg(time));
    QString id = cloud->id();
    cloud->setId(id + KEYPOINTS_PRE_FLAG);
    m_cloudview->addPointCloud(cloud);
    m_cloudview->setPointCloudColor(cloud->id(), QColorConstants::Green);
    m_cloudview->setPointCloudSize(cloud->id(), cloud->pointSize() + 2);
    m_keypoints_map[id] = cloud;
    m_cloudtree->closeProgressBar();
}

#include "segmentation.h"
#include "ui_segmentation.h"

#define SEG_TYPE_SACSegmentation                (0)
#define SEG_TYPE_EuclideanClusterExtraction     (1)
#define SEG_TYPE_RegionGrowing                  (2)
#define SEG_TYPE_SupervoxelClustering           (3)

#define SEG_PRE_FLAG                            "-seged"
#define SEG_ADD_FLAG                            "seged-"

Segmentation::Segmentation(QWidget* parent) :
    CustomDock(parent), ui(new Ui::Segmentation),
    m_thread(this)
{
    ui->setupUi(this);

    qRegisterMetaType<Eigen::Vector3f>("Eigen::Vector3f &");
    qRegisterMetaType<Eigen::Vector3f>("Eigen::Vector3f");
    qRegisterMetaType<ct::ModelCoefficients::Ptr>("ct::ModelCoefficients::Ptr &");
    qRegisterMetaType<ct::ModelCoefficients::Ptr>("ct::ModelCoefficients::Ptr");
    qRegisterMetaType<std::vector<ct::Cloud::Ptr>>("std::vector<ct::Cloud::Ptr> &");
    qRegisterMetaType<std::vector<ct::Cloud::Ptr>>("std::vector<ct::Cloud::Ptr>");

    connect(ui->btn_preview, &QPushButton::clicked, this, &Segmentation::preview);
    connect(ui->btn_add, &QPushButton::clicked, this, &Segmentation::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Segmentation::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Segmentation::reset);

    m_seg = new ct::Segmentation;
    m_seg->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_seg, &QObject::deleteLater);
    connect(this, &Segmentation::SACSegmentation, m_seg, &ct::Segmentation::SACSegmentation);
    connect(this, &Segmentation::SACSegmentationFromNormals, m_seg, &ct::Segmentation::SACSegmentationFromNormals);
    connect(this, &Segmentation::EuclideanClusterExtraction, m_seg, &ct::Segmentation::EuclideanClusterExtraction);
    connect(this, &Segmentation::RegionGrowing, m_seg, &ct::Segmentation::RegionGrowing);
    connect(this, &Segmentation::RegionGrowingRGB, m_seg, &ct::Segmentation::RegionGrowingRGB);
    connect(this, &Segmentation::SupervoxelClustering, m_seg, &ct::Segmentation::SupervoxelClustering);
    connect(m_seg, &ct::Segmentation::segmentationResult, this, &Segmentation::segmentationResult);
    m_thread.start();

    // connect(ui->cbox_Segmentation, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [=](int index)
    //         {
    //             if (index == 0 || index == 6)
    //                 ui->check_negative->setEnabled(true);
    //             else
    //             {
    //                 ui->check_negative->setChecked(false);
    //                 ui->check_negative->setEnabled(false);
    //             }
    //         });

    //SACSegmentation
    connect(ui->checkbox_fromNormals, &QCheckBox::clicked, [=](bool state)
            {
                if (state)
                    ui->dspin_distanceWeight->setEnabled(true);
                else
                    ui->dspin_distanceWeight->setEnabled(false);
            });

    //RegionGrowing
    connect(ui->check_smoothmode, &QCheckBox::clicked, [=](bool state)
            {
                if (state) ui->dspin_smooth->setEnabled(true);
                else ui->dspin_smooth->setEnabled(false);
            });
    connect(ui->check_curvaturetest, &QCheckBox::clicked, [=](bool state)
            {
                if (state) ui->dspin_curvature->setEnabled(true);
                else ui->dspin_curvature->setEnabled(false);
            });
    connect(ui->check_residualtest, &QCheckBox::clicked, [=](bool state)
            {
                if (state) ui->dspin_residual->setEnabled(true);
                else ui->dspin_residual->setEnabled(false);
            });
    ui->cbox_segmentations->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
}

Segmentation::~Segmentation()
{
    m_thread.quit();
    m_thread.wait();
    delete ui;
}

void Segmentation::preview()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    // for (size_t i = 0;i < segmented_clouds.size();i++)
    //     cloud_view->removeCloud("segmented_" + std::to_string(i));
    // segmented_clouds.clear();
    switch (ui->cbox_segmentations->currentIndex())
    {
    case SEG_TYPE_SACSegmentation:
        m_cloudview->showInfo("SACSegmentation", 1);
        //TODO:
        break;
    case SEG_TYPE_EuclideanClusterExtraction:
        m_cloudview->showInfo("EuclideanClusterExtraction", 1);
        //TODO:    
        break;
    case SEG_TYPE_RegionGrowing:
        m_cloudview->showInfo("RegionGrowing", 1);
        //TODO:  
        break;
    case SEG_TYPE_SupervoxelClustering:
        m_cloudview->showInfo("SupervoxelClustering", 1);
        //TODO:  
        break;
    }
    m_cloudtree->showProgressBar();
}

void Segmentation::add()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_segmentation_map.find(cloud->id()) == m_segmentation_map.end())
        {
            printW(QString("The cloud[id:%1] has no seged clouds !").arg(cloud->id()));
            continue;
        }
        std::vector<ct::Cloud::Ptr> new_clouds = m_segmentation_map.find(cloud->id())->second;
        for (int i = 0;i < new_clouds.size();i++)
        {
            m_cloudview->removePointCloud(new_clouds[i]->id());
            new_clouds[i]->setId(SEG_ADD_FLAG + QString::number(i) + "-" + cloud->id());
            m_cloudtree->appendCloud(cloud, new_clouds[i], true);
            printI(QString("Add seged cloud[id:%1] done.").arg(new_clouds[i]->id()));
        }
        m_segmentation_map.erase(cloud->id());
    }
    m_cloudview->clearInfo();
}

void Segmentation::apply()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    for (auto& cloud : selected_clouds)
    {
        if (m_segmentation_map.find(cloud->id()) == m_segmentation_map.end())
        {
            printW(QString("The cloud[id:%1] has no segmented cloud !").arg(cloud->id()));
            continue;
        }
        std::vector<ct::Cloud::Ptr> new_clouds = m_segmentation_map.find(cloud->id())->second;
        for (int i = 0;i < new_clouds.size();i++)
        {
            m_cloudview->removePointCloud(new_clouds[i]->id());
            new_clouds[i]->setId(SEG_ADD_FLAG + QString::number(i) + "-" + cloud->id());
            if (i == 0)
            {
                m_cloudtree->updateCloud(cloud, new_clouds[i], true);
                printI(QString("Apply segmented cloud[id:%1] done.").arg(new_clouds[i]->id()));
            }
            else
            {
                m_cloudtree->appendCloud(cloud, new_clouds[i], true);
                printI(QString("Add segmented cloud[id:%1] done.").arg(new_clouds[i]->id()));
            }
        }
        m_segmentation_map.erase(cloud->id());
    }
    m_cloudview->clearInfo();
}

void Segmentation::reset()
{
    for (auto& cloud : m_cloudtree->getSelectedClouds())
        m_cloudtree->setCloudChecked(cloud);
    for (auto& clouds : m_segmentation_map)
        for (auto& cloud : clouds.second)
            m_cloudview->removePointCloud(cloud->id());
    m_segmentation_map.clear();
    m_cloudview->clearInfo();
}

void Segmentation::segmentationResult(const QString& id, const std::vector<ct::Cloud::Ptr>& cloud, float time, const ct::ModelCoefficients::Ptr& cofe)
{
    printI(QString("Segmented cloud[%1] to %2 cloud(s) done, take time %2 ms.").arg(id).arg(cloud.size()).arg(time));
    for (size_t i = 0;i < cloud.size();i++)
    {
        if (cloud[i]->points.size() <= 0)continue;
        cloud[i]->setId(id + SEG_PRE_FLAG + QString::number(i));
        m_cloudview->addPointCloud(cloud[i]);
        m_cloudview->setPointCloudColor(cloud[i]->id(), QColor(rand() % 256, rand() % 256, rand() % 256));
        m_cloudview->setPointCloudSize(cloud[i]->id(), cloud[i]->pointSize() + 2);
    }
    m_segmentation_map[id] = cloud;
    m_cloudtree->closeProgressBar();
}
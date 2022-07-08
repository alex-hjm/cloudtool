#include "segmentation.h"
#include "ui_segmentation.h"

#define SEG_TYPE_SACSegmentation                (0)
#define SEG_TYPE_EuclideanClusterExtraction     (1)
#define SEG_TYPE_RegionGrowing                  (2)
#define SEG_TYPE_SupervoxelClustering           (3)

#define SEG_PRE_FLAG                            "-seg"
#define SEG_ADD_FLAG                            "seg-"

Segmentation::Segmentation(QWidget* parent) :
    CustomDock(parent), ui(new Ui::Segmentation),
    m_thread(this)
{
    ui->setupUi(this);

    qRegisterMetaType<ct::ModelCoefficients::Ptr>("ModelCoefficients::Ptr &");
    qRegisterMetaType<ct::ModelCoefficients::Ptr>("ModelCoefficients::Ptr");
    qRegisterMetaType<std::vector<ct::Cloud::Ptr>>("std::vector<Cloud::Ptr> &");
    qRegisterMetaType<std::vector<ct::Cloud::Ptr>>("std::vector<Cloud::Ptr>");

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

    //TODO:
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
    connect(ui->check_fromNormals, &QCheckBox::clicked, [=](bool state)
            {
                if (state)
                {
                    ui->label_5->show(), ui->dspin_distanceWeight->show();
                    ui->label_37->show(), ui->dspin_distanceFromOrigin->show();
                }
                else
                {
                    ui->label_5->hide(), ui->dspin_distanceWeight->hide();
                    ui->label_37->hide(), ui->dspin_distanceFromOrigin->hide();
                }
            });
    connect(ui->cbox_modelType, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), [=](int i)
            {
                if (i == 2 || i == 3 || i == 4 || i == 5 || i == 6 || i == 7 || i == 12)
                    ui->label_7->show(), ui->dspin_minRadius->show(), ui->dspin_maxRadius->show();
                else
                    ui->label_7->hide(), ui->dspin_minRadius->hide(), ui->dspin_maxRadius->hide();
            });

    ui->check_fromNormals->setChecked(false);
    ui->label_5->hide(), ui->dspin_distanceWeight->hide();
    ui->label_37->hide(), ui->dspin_distanceFromOrigin->hide();
    ui->label_7->hide(), ui->dspin_minRadius->hide(), ui->dspin_maxRadius->hide();

    //RegionGrowing
    connect(ui->check_smoothmode, &QCheckBox::clicked, [=](bool state)
            {
                ui->dspin_smooth->setEnabled(state);
            });
    connect(ui->check_curvaturetest, &QCheckBox::clicked, [=](bool state)
            {
                ui->dspin_curvature->setEnabled(state);
            });
    connect(ui->check_residualtest, &QCheckBox::clicked, [=](bool state)
            {
                ui->dspin_residual->setEnabled(state);
            });
    connect(ui->check_fromRGB, &QCheckBox::clicked, [=](bool state)
            {
                if (state)
                {
                    ui->label_14->show(), ui->dspin_pointcolor->show();
                    ui->label_17->show(), ui->dspin_regioncolor->show();
                    ui->label_15->show(), ui->dspin_distance->show();
                    ui->label_16->show(), ui->spin_nghbr_number->show();
                }
                else
                {
                    ui->label_14->hide(), ui->dspin_pointcolor->hide();
                    ui->label_17->hide(), ui->dspin_regioncolor->hide();
                    ui->label_15->hide(), ui->dspin_distance->hide();
                    ui->label_16->hide(), ui->spin_nghbr_number->hide();
                }
            });

    ui->check_fromRGB->setChecked(false);
    ui->label_14->hide(), ui->dspin_pointcolor->hide();
    ui->label_17->hide(), ui->dspin_regioncolor->hide();
    ui->label_15->hide(), ui->dspin_distance->hide();
    ui->label_16->hide(), ui->spin_nghbr_number->hide();
    ui->check_residualtest->setChecked(false);
    ui->dspin_residual->setEnabled(false);


    ui->cbox_segmentations->setCurrentIndex(0);
    ui->stackedWidget->setCurrentIndex(0);
}

Segmentation::~Segmentation()
{
    m_thread.quit();
    if (!m_thread.wait(3000))
    {
        m_thread.terminate();
        m_thread.wait();
    }
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
    for (auto& clouds : m_segmentation_map)
        for (auto& cloud : clouds.second)
            m_cloudview->removePointCloud(cloud->id());
    for (auto& cloud : selected_clouds)
    {
        m_seg->setInputCloud(cloud);
        m_seg->setNegative(ui->check_negative->isChecked());
        switch (ui->cbox_segmentations->currentIndex())
        {
        case SEG_TYPE_SACSegmentation:
            m_cloudview->showInfo("SACSegmentation", 1);
            if (ui->check_fromNormals->isChecked())
            {
                if (!cloud->hasNormals())
                {
                    printW("Please estimate normals first!");
                    return;
                }
                emit SACSegmentationFromNormals(ui->cbox_modelType->currentIndex(), ui->cbox_methodType->currentIndex(), ui->dspin_Threshold->value(),
                                                ui->spin_Iterations->value(), ui->dspin_probability->value(), ui->check_optimize->isChecked(), ui->dspin_minRadius->value(),
                                                ui->dspin_maxRadius->value(), ui->dspin_distanceWeight->value(), ui->dspin_distanceFromOrigin->value());
            }

            else
                emit SACSegmentation(ui->cbox_modelType->currentIndex(), ui->cbox_methodType->currentIndex(), ui->dspin_Threshold->value(),
                                     ui->spin_Iterations->value(), ui->dspin_probability->value(), ui->check_optimize->isChecked(), ui->dspin_minRadius->value(),
                                     ui->dspin_maxRadius->value());

            break;
        case SEG_TYPE_EuclideanClusterExtraction:
            m_cloudview->showInfo("EuclideanClusterExtraction", 1);
            emit EuclideanClusterExtraction(ui->dspin_tolerance->value(), ui->spin_min_cluster_size->value(), ui->spin_max_cluster_size->value());
            break;
        case SEG_TYPE_RegionGrowing:
            m_cloudview->showInfo("RegionGrowing", 1);
            if (!cloud->hasNormals())
            {
                printW("Please estimate normals first!");
                return;
            }
            if (ui->check_fromRGB->isChecked())
            {
                if (cloud->type() == CLOUD_TYPE_XYZ || cloud->type() == CLOUD_TYPE_XYZN)
                {
                    printW("The cloud type does not support !");
                    return;
                }
                emit RegionGrowingRGB(ui->spin_minclustersize->value(), ui->spin_maxclustersize->value(), ui->check_smoothmode->isChecked(),
                                      ui->check_curvaturetest->isChecked(), ui->check_residualtest->isChecked(), ui->dspin_smooth->value(),
                                      ui->dspin_residual->value(), ui->dspin_curvature->value(), ui->spin_numofnei->value(), ui->dspin_pointcolor->value(),
                                      ui->dspin_regioncolor->value(), ui->dspin_distance->value(), ui->spin_nghbr_number->value());
            }
            else
                emit RegionGrowing(ui->spin_minclustersize->value(), ui->spin_maxclustersize->value(), ui->check_smoothmode->isChecked(),
                                   ui->check_curvaturetest->isChecked(), ui->check_residualtest->isChecked(), ui->dspin_smooth->value(),
                                   ui->dspin_residual->value(), ui->dspin_curvature->value(), ui->spin_numofnei->value());

            //TODO:  
            break;
        case SEG_TYPE_SupervoxelClustering:
            m_cloudview->showInfo("SupervoxelClustering", 1);
            emit SupervoxelClustering(ui->dspin_voxelresolution->value(), ui->dspin_seedresolution->value(), ui->dspin_colorimportance->value(),
                                      ui->dspin_spatialmportance->value(), ui->dspin_normallmportance->value(), ui->check_transform->isChecked());
            break;
        }
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
        QString id = cloud->id();
        std::vector<ct::Cloud::Ptr> new_clouds = m_segmentation_map.find(id)->second;
        for (int i = 0;i < new_clouds.size();i++)
        {
            m_cloudview->removePointCloud(new_clouds[i]->id());
            new_clouds[i]->setId(SEG_ADD_FLAG + QString::number(i) + "-" + id);
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
        m_segmentation_map.erase(id);
    }
    m_cloudview->clearInfo();
}

void Segmentation::reset()
{
    for (auto& clouds : m_segmentation_map)
        for (auto& cloud : clouds.second)
            m_cloudview->removePointCloud(cloud->id());
    m_segmentation_map.clear();
    m_cloudview->clearInfo();
}

void Segmentation::segmentationResult(const QString& id, const std::vector<ct::Cloud::Ptr>& cloud, float time, const ct::ModelCoefficients::Ptr&)
{
    printI(QString("Segmented cloud[%1] to %2 cloud(s) done, take time %3 ms.").arg(id).arg(cloud.size()).arg(time));
    for (size_t i = 0;i < cloud.size();i++)
    {
        if (cloud[i]->points.size() <= 0)continue;
        cloud[i]->setId(id + SEG_PRE_FLAG + QString::number(i));
        m_cloudview->addPointCloud(cloud[i]);
        m_cloudview->setPointCloudColor(cloud[i]->id(), {rand() % 256, rand() % 256, rand() % 256});
        m_cloudview->setPointCloudSize(cloud[i]->id(), cloud[i]->pointSize() + 2);
    }
    m_segmentation_map[id] = cloud;
    m_cloudtree->closeProgressBar();
}
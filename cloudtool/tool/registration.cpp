#include "registration.h"
#include "ui_registration.h"

#define SEG_TYPE_CorrespondenceEstimation       (0)
#define SEG_TYPE_CorrespondenceRejector         (1)
#define SEG_TYPE_TransformationEstimation       (2)
#define SEG_TYPE_Registration                   (3)

#define SEG_TYPE_CE_Base                        (0)
#define SEG_TYPE_CE_BackProjection              (1)
#define SEG_TYPE_CE_NormalShooting              (2)

#define SEG_TYPE_PointCloud                     (0)
#define SEG_TYPE_PFHFeature                     (1)
#define SEG_TYPE_FPFHFeature                    (2)

#define SEG_CORRE_PRE_FLAG                      "-corre"

Registration::Registration(QWidget* parent)
    :CustomDock(parent), ui(new Ui::Registration),
    m_thread(this), m_target_cloud(nullptr), m_source_cloud(nullptr)
{
    ui->setupUi(this);
    connect(ui->btn_setTarget, &QPushButton::clicked, this, &Registration::setTarget);
    connect(ui->btn_setSource, &QPushButton::clicked, this, &Registration::setSource);
    connect(ui->btn_preview, &QPushButton::clicked, this, &Registration::preview);
    connect(ui->btn_add, &QPushButton::clicked, this, &Registration::add);
    connect(ui->btn_apply, &QPushButton::clicked, this, &Registration::apply);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Registration::reset);

    m_reg = new ct::Registration;
    m_reg->moveToThread(&m_thread);
    connect(&m_thread, &QThread::finished, m_reg, &QObject::deleteLater);
    connect(m_reg, &ct::Registration::correspondenceEstimationResult, this, &Registration::correspondenceEstimationResult);
    m_thread.start();

    ui->cbox_type->setCurrentIndex(0);
    ui->cbox_feature->setCurrentIndex(0);
    ui->cbox_correEst->setCurrentIndex(0);

    ui->stackedWidget->setCurrentIndex(0);
    ui->stackedWidget_2->setCurrentIndex(0);
    ui->stackedWidget_3->setCurrentIndex(0);
}

Registration::~Registration()
{
    m_thread.quit();
    m_thread.wait();
    delete ui;
}

void Registration::setTarget()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    m_target_cloud = selected_clouds.front();
    if (m_source_cloud && m_source_cloud->id() == m_target_cloud->id())
    {
        printW("Please choose another cloud as target cloud!");
        return;
    }
    m_cloudview->removeShape(m_target_cloud->boxId());
    m_cloudview->setPointCloudColor(m_target_cloud, QColorConstants::Red);
    m_cloudview->showInfo("Target Cloud(Red): " + m_target_cloud->id(), 1);
    if (m_source_cloud)
    {
        ui->btn_setTarget->setEnabled(false);
        ui->btn_setSource->setEnabled(false);
        m_cloudtree->setEnabled(false);
    }
}

void Registration::setSource()
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        printW("Please select a cloud!");
        return;
    }
    m_source_cloud = selected_clouds.front();
    if (m_target_cloud && m_target_cloud->id() == m_source_cloud->id())
    {
        printW("Please choose another cloud as target cloud!");
        return;
    }
    m_cloudview->removeShape(m_source_cloud->boxId());
    m_cloudview->setPointCloudColor(m_source_cloud, QColorConstants::Green);
    m_cloudview->showInfo("Source Cloud(Green): " + m_source_cloud->id(), 2);
    if (m_target_cloud)
    {
        ui->btn_setTarget->setEnabled(false);
        ui->btn_setSource->setEnabled(false);
        m_cloudtree->setEnabled(false);
    }
}

void Registration::preview()
{
    if (!m_source_cloud || !m_target_cloud)
    {
        printW("Please select a source cloud and a target cloud!");
        return;
    }
    switch (ui->cbox_type->currentIndex())
    {
    case SEG_TYPE_CorrespondenceEstimation:
        switch (ui->cbox_correEst->currentIndex())
        {
        case SEG_TYPE_CE_Base:
            if (ui->cbox_ce_type->currentIndex() > 0 && m_descriptor == nullptr)
            {
                printW("Please estimate cloud descriptor first!");
                return;
            }
            switch (ui->cbox_ce_type->currentIndex())
            {
            case SEG_TYPE_PointCloud:
                m_reg->CorrespondenceEstimation<ct::PointXYZRGBN>(m_source_cloud, m_target_cloud);
                break;
            case SEG_TYPE_PFHFeature:
                if (m_descriptor->getDescriptor(m_source_cloud->id())->pfh && m_descriptor->getDescriptor(m_target_cloud->id())->pfh)
                    m_reg->CorrespondenceEstimation<pcl::PFHSignature125>(m_descriptor->getDescriptor(m_source_cloud->id())->pfh,
                                                                          m_descriptor->getDescriptor(m_target_cloud->id())->pfh);
                else
                    printW("Please estimate cloud PFHFeature first!");
                break;
            case SEG_TYPE_FPFHFeature:
                if (m_descriptor->getDescriptor(m_source_cloud->id())->fpfh && m_descriptor->getDescriptor(m_target_cloud->id())->fpfh)
                    m_reg->CorrespondenceEstimation<pcl::FPFHSignature33>(m_descriptor->getDescriptor(m_source_cloud->id())->fpfh,
                                                                          m_descriptor->getDescriptor(m_target_cloud->id())->fpfh);
                else
                    printW("Please estimate cloud FPFHFeature first!");
                break;
            }
            break;
        case SEG_TYPE_CE_BackProjection:
            break;
        case SEG_TYPE_CE_NormalShooting:
            break;
        }
        break;
    case SEG_TYPE_CorrespondenceRejector:

        break;
    case SEG_TYPE_TransformationEstimation:

        break;
    case SEG_TYPE_Registration:

        break;
    }
}

void Registration::add()
{}

void Registration::apply()
{}

void Registration::reset()
{
    m_cloudtree->setEnabled(true);
    ui->btn_setSource->setEnabled(true);
    ui->btn_setTarget->setEnabled(true);
    m_cloudview->clearInfo();
    if(m_target_cloud) m_cloudview->resetPointCloudColor(m_target_cloud);
    if(m_source_cloud) m_cloudview->resetPointCloudColor(m_source_cloud);
    m_target_cloud = nullptr;
    m_source_cloud = nullptr;
}

void Registration::correspondenceEstimationResult(const ct::CorrespondencesPtr& corr, float time)
{
    printI(QString("Estimate cloud[id:%1] and cloud[id:%2] correspondences done, take time %3 ms.").arg(m_target_cloud->id()).arg(m_source_cloud->id()).arg(time));
    QString id = m_target_cloud->id() + m_source_cloud->id() + SEG_CORRE_PRE_FLAG;
    switch (ui->cbox_correEst->currentIndex())
    {
    case SEG_TYPE_CE_Base:
        switch (ui->cbox_ce_type->currentIndex())
        {
        case SEG_TYPE_PointCloud:
            m_cloudview->addCorrespondences<ct::PointXYZRGBN>(m_source_cloud, m_target_cloud, corr, id);
            break;
        case SEG_TYPE_PFHFeature:
            m_cloudview->addCorrespondences<ct::PointXYZRGBN>(m_source_cloud, m_target_cloud, corr, id);
            // m_cloudview->addCorrespondences<pcl::PFHSignature125>(m_descriptor->getDescriptor(m_source_cloud->id())->pfh,
            //                                                       m_descriptor->getDescriptor(m_target_cloud->id())->pfh, corr, id);
            break;
        case SEG_TYPE_FPFHFeature:
            m_cloudview->addCorrespondences<ct::PointXYZRGBN>(m_source_cloud, m_target_cloud, corr, id);
            // m_cloudview->addCorrespondences<pcl::FPFHSignature33>(m_descriptor->getDescriptor(m_source_cloud->id())->fpfh,
            //                                                       m_descriptor->getDescriptor(m_target_cloud->id())->fpfh, corr, id);
            break;
        }
        break;
    case SEG_TYPE_CE_BackProjection:
        break;
    case SEG_TYPE_CE_NormalShooting:
        break;
    }
}


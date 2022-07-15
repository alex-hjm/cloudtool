/**
 * @file azurekinect.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#include "azurekinect.h"
#include "ui_azurekinect.h"

#define KINECT_PRE_FLAG "azurekinect-preview"
#define KINECT_ADD_FLAG "azurekinect-pointcloud"

AzureKinect::AzureKinect(QWidget* parent)
    : CustomDock(parent), ui(new Ui::AzureKinect),
    m_captured_cloud(new ct::Cloud)
{
    ui->setupUi(this);

    connect(ui->btn_connect, &QPushButton::clicked, this, &AzureKinect::connectDevice);
    connect(ui->btn_capture, &QPushButton::clicked, this, &AzureKinect::captureDevice);
    connect(ui->btn_add, &QPushButton::clicked, this, &AzureKinect::add);
    connect(ui->btn_reset, &QPushButton::clicked, this, &AzureKinect::reset);

    ui->cbox_image_format->setCurrentIndex(0);
    ui->cbox_resolution->setCurrentIndex(0);
    ui->cbox_depth_mode->setCurrentIndex(0);
    ui->cbox_fps->setCurrentIndex(2);

}

AzureKinect::~AzureKinect()
{
    delete ui;
}

void AzureKinect::connectDevice()
{
    if (!m_device.is_valid())
    {
        uint32_t dev_cnt = k4a::device::get_installed_count();
        if (dev_cnt <= 0)
        {
            printE("No K4A devices found.");
            return;
        }
        m_device = k4a::device::open(K4A_DEVICE_DEFAULT);
        if(!m_device.is_valid())
        {
            printW(tr("K4A devices(%1) open failed.").arg(K4A_DEVICE_DEFAULT));
            return;
        }
        printI(tr("Open k4a devices(%1) successfully.").arg(m_device.get_serialnum().c_str()));
    }
}

void AzureKinect::captureDevice()
{

}

void AzureKinect::add()
{
    if (m_captured_cloud->empty())
    {
        printW("Please capture a cloud first!");
        return;
    }
    m_cloudview->removePointCloud(KINECT_PRE_FLAG);
    ct::Cloud::Ptr new_cloud = m_captured_cloud->makeShared();
    new_cloud->setId(KINECT_ADD_FLAG);
    new_cloud->setInfo(QFileInfo(QString(DATA_PATH) + QString(KINECT_ADD_FLAG)));
    new_cloud->update(false);
    m_cloudtree->appendCloud(new_cloud, true);
    m_captured_cloud->clear();
    printI(QString("Add captured cloud[id:%1] done.").arg(m_captured_cloud->id()));
}

void AzureKinect::reset()
{
    m_cloudview->removePointCloud(KINECT_PRE_FLAG);
    m_captured_cloud->clear();
}


/**
 * @file photoneo.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#include "photoneo.h"
#include "ui_photoneo.h"

#include "PhoXiPCLSupport.h"
#include "PhoXiDataTypes.h"

#define PHOTONEO_PRE_FLAG "photoneo-preview"
#define PHOTONEO_ADD_FLAG "photoneo-pointcloud"

Photoneo::Photoneo(QWidget* parent) : CustomDock(parent), ui(new Ui::Photoneo),
m_captured_cloud(new ct::Cloud)
{
    ui->setupUi(this);

    connect(ui->btn_search, &QPushButton::clicked, this, &Photoneo::searchDevice);
    connect(ui->btn_connect, &QPushButton::clicked, this, &Photoneo::connectDevice);
    connect(ui->btn_capture, &QPushButton::clicked, this, &Photoneo::captureDevice);
    connect(ui->btn_add, &QPushButton::clicked, this, &Photoneo::add);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Photoneo::reset);

    ui->cbox_device->setCurrentIndex(-1);
    connect(ui->cbox_device, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Photoneo::updateDeviceInfo);
}

Photoneo::~Photoneo()
{
    if (m_device)
        m_device->Disconnect(true, true);
    delete ui;
}

void Photoneo::searchDevice()
{
    if (!m_factory.isPhoXiControlRunning())
    {
        printE("PhoXi Control Software is not running");
        return;
    }
    m_device_list = m_factory.GetDeviceList();
    if (m_device_list.empty())
    {
        printE("PhoXi Factory has found 0 devices");
        return;
    }
    ui->cbox_device->clear();
    for (auto device : m_device_list)
    {
        ui->cbox_device->addItem(device.Name.c_str());
    }
    ui->cbox_device->setCurrentIndex(0);
    this->updateDeviceInfo(0);
}

void Photoneo::connectDevice()
{
    if (!m_factory.isPhoXiControlRunning())
    {
        printW("PhoXi Control Software is not running");
        return;
    }
    if (!m_device)
    {
        if (m_device_list.empty())
        {
            printW("Device list is empty");
            return;
        }
        int index = ui->cbox_device->currentIndex();
        if (index < 0 || index > m_device_list.size() - 1)
        {
            printW("Selected device is invaild.");
            return;
        }
        m_device = m_factory.Create(m_device_list[index]);
        if (!m_device)
        {
            printE(tr("Device: %1 was not created").arg(m_device_list[index].HWIdentification.c_str()));
            return;
        }
        if (!m_device->Connect())
        {
            printE(tr("Connect to the device: %1 failed.").arg(m_device_list[index].HWIdentification.c_str()));
            return;
        }
        printI(tr("Connect to the device: %1 successfully.").arg(m_device_list[index].HWIdentification.c_str()));
        ui->btn_connect->setIcon(QIcon(":/res/icon/disconnect.svg"));
    }
    else
    {
        if (m_device->isConnected())
        {
            if (!m_device->Disconnect())
            {
                printE(tr("Disconnect the device: %1 failed.").arg(m_device->GetName().c_str()));
                return;
            }
            printI(tr("Disconnect the device: %1 successfully.").arg(m_device->GetName().c_str()));
            ui->btn_connect->setIcon(QIcon(":/res/icon/connect.svg"));
        }
        else
        {
            if (!m_device->Connect())
            {
                printE(tr("Connect to the device: %1 failed.").arg(m_device->GetName().c_str()));
                return;
            }
            printI(tr("Connect to the device: %1 successfully.").arg(m_device->GetName().c_str()));
            ui->btn_connect->setIcon(QIcon(":/res/icon/disconnect.svg"));
        }
    }
}

void Photoneo::captureDevice()
{
    if (!m_device || !m_device->isConnected())
    {
        printW("Please connect a device first!");
        return;
    }
    if (m_device->isAcquiring())
    {
        m_device->StopAcquisition();
    }
    m_device->TriggerMode = pho::api::PhoXiTriggerMode::Software;
    printI("Software trigger mode was set");
    m_device->ClearBuffer();
    m_device->StartAcquisition();
    if (!m_device->isAcquiring())
    {
        printE("Your device could not start acquisition!");
        return;
    }
    printI("Start trigger the frame");
    ct::TicToc ticToc;
    ticToc.tic();
    int FrameID = m_device->TriggerFrame();
    if (FrameID < 0)
    {
        printE("Trigger was unsuccessful!");
        return;
    }
    printI(tr("Frame was triggered, Frame Id: %1").arg(FrameID));
    pho::api::PFrame Frame = m_device->GetSpecificFrame(FrameID, pho::api::PhoXiTimeout::Infinity);
    if (Frame)
        updatePointCloud(Frame, ticToc.toc());
    else
        printE("Failed to retrieve the frame!");
    m_device->StopAcquisition();
}

void Photoneo::add()
{
    if (m_captured_cloud->empty())
    {
        printW("Please capture a cloud first!");
        return;
    }
    m_cloudview->removePointCloud(PHOTONEO_PRE_FLAG);
    ct::Cloud::Ptr new_cloud = m_captured_cloud->makeShared();
    new_cloud->setId(PHOTONEO_ADD_FLAG);
    new_cloud->setInfo(QFileInfo(QString(DATA_PATH) + QString(PHOTONEO_ADD_FLAG)));
    new_cloud->update(false);
    m_cloudtree->appendCloud(new_cloud, true);
    m_captured_cloud->clear();
    printI(QString("Add captured cloud[id:%1] done.").arg(m_captured_cloud->id()));
}

void Photoneo::reset()
{
    m_cloudview->removePointCloud(PHOTONEO_PRE_FLAG);
    m_captured_cloud->clear();
}

void Photoneo::updateDeviceInfo(int index)
{
    ui->txt_info->clear();
    if (index < 0) return;
    if (m_device_list.empty())
    {
        printW("Device List is empty!");
        return;
    }
    if (index > m_device_list.size() - 1)
    {
        printW("Device is not exist, please search device again!");
        return;
    }
    pho::api::PhoXiDeviceInformation info = m_device_list[index];
    ui->txt_info->append(tr("Name:%1").arg(info.Name.c_str()));
    ui->txt_info->append(tr("Hardware Identification:%1").arg(info.HWIdentification.c_str()));
    ui->txt_info->append(tr("Type:%1").arg(info.Type));
    ui->txt_info->append(tr("Firmware version:%1").arg(info.FirmwareVersion.c_str()));
    ui->txt_info->append(tr("Variant:%1").arg(info.Variant.c_str()));
    ui->txt_info->append(tr("IsFileCamera:%1").arg(info.IsFileCamera ? "Yes" : "No"));
    ui->txt_info->append(tr("Status:%1").arg(info.Status.Attached ?
                                             "Attached to PhoXi Control. " : "Not Attached to PhoXi Control." +
                                             info.Status.Ready ? "Ready to connect" : "Occupied"));
}

void Photoneo::updatePointCloud(const pho::api::PFrame& frame, float time)
{
    pho::api::PhoXiTimeout timeout;
    frame->ConvertTo(*m_captured_cloud);
    m_captured_cloud->setId(PHOTONEO_PRE_FLAG);
    printI(tr("Number of points in PCL Cloud : %1").arg(m_captured_cloud->size()));
    printI(tr("Capture cloud[id:%1] done, take time %2 ms.").arg(m_captured_cloud->id()).arg(time));
    m_cloudview->addPointCloud(m_captured_cloud);
    m_cloudview->resetCamera();
}
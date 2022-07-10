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

#define PHOTONEO_CAP_FLAG   "photoneo"

PhotoneoGrabber::~PhotoneoGrabber()
{
    if (!m_device) return;
    if (!m_device->isConnected()) return;
    if (m_device->isAcquiring()) m_device->StopAcquisition();
    m_device->Disconnect();
}

bool PhotoneoGrabber::connectDevice()
{
    pho::api::PhoXiFactory factory;
    if (!factory.isPhoXiControlRunning())
    {
        emit printE("PhoXi Control Software is not running");
        return false;
    }
    std::vector <pho::api::PhoXiDeviceInformation> DeviceList = factory.GetDeviceList();
    if (DeviceList.empty())
    {
        emit printE("PhoXi Factory has found 0 devices");
        return false;
    }
    this->printDeviceInfoList(DeviceList);
    m_device = factory.CreateAndConnectFirstAttached();
    if (m_device)
    {
        emit printI(tr("You have already PhoXi device(%1) opened in PhoXi Control.").
                    arg(((std::string)m_device->HardwareIdentification).c_str()));
    }
    else
    {
        emit printI("You have no PhoXi device opened in PhoXi Control, connect to last device in device list");
        m_device = factory.CreateAndConnect(DeviceList.back().HWIdentification);
    }
    if (!m_device)
    {
        emit printE("Your device was not created!");
        return false;
    }
    if (!m_device->isConnected())
    {
        emit printE("Your device is not connected");
        return false;
    }
    return true;
}

bool PhotoneoGrabber::disconnectDevice()
{
    if (!m_device)
    {
        emit printE("Your device was not created!");
        return false;
    }
    if (!m_device->isConnected())
    {
        emit printE("Your device is not connected");
        return false;
    }
    m_device->Disconnect();
    return true;
}

bool PhotoneoGrabber::captureOnce()
{
    if (m_device->isAcquiring())
    {
        m_device->StopAcquisition();
    }
    m_device->TriggerMode = pho::api::PhoXiTriggerMode::Software;
    emit printI("Software trigger mode was set");
    m_device->ClearBuffer();
    m_device->StartAcquisition();
    if (!m_device->isAcquiring())
    {
        emit printE("Your device could not start acquisition!");
        return false;
    }
    emit printI("Start trigger the frame");
    ct::TicToc ticToc;
    ticToc.tic();
    int FrameID = m_device->TriggerFrame();
    if (FrameID < 0)
    {
        emit printE("Trigger was unsuccessful!");
        return false;
    }
    else
    {
        emit printI(tr("Frame was triggered, Frame Id: %1").arg(FrameID));
    }
    pho::api::PFrame Frame = m_device->GetSpecificFrame(FrameID, pho::api::PhoXiTimeout::Infinity);
    if (Frame)
    {
        pho::api::PhoXiTimeout timeout;
        ct::Cloud::Ptr capture_cloud(new ct::Cloud);
        Frame->ConvertTo(*capture_cloud);
        capture_cloud->setId(PHOTONEO_CAP_FLAG);
        emit printI(tr("Number of points in PCL Cloud : %1").arg(capture_cloud->size()));
        emit sendPointcloud(capture_cloud, ticToc.toc());
    }
    else
    {
        emit printE("Failed to retrieve the frame!");
    }
    m_device->StopAcquisition();
    return true;
}

bool PhotoneoGrabber::isCapturing()
{
    if (!m_device) return false;
    return m_device->isAcquiring();
}

bool PhotoneoGrabber::isConnected()
{
    if (!m_device) return false;
    return m_device->isConnected();
}

void PhotoneoGrabber::printDeviceInfoList(const std::vector<pho::api::PhoXiDeviceInformation>& DeviceList)
{
    for (std::size_t i = 0; i < DeviceList.size(); ++i)
    {
        emit printI(tr("Device: %1").arg(i));
        this->printDeviceInfo(DeviceList[i]);
    }
}

void PhotoneoGrabber::printDeviceInfo(const pho::api::PhoXiDeviceInformation& DeviceInfo)
{
    emit printI(tr("Name:                    %1").arg(DeviceInfo.Name.c_str()));
    emit printI(tr("Hardware Identification: %1").arg(DeviceInfo.HWIdentification.c_str()));
    emit printI(tr("Type:                    %1").arg(DeviceInfo.Type));
    emit printI(tr("Firmware version:        %1").arg(DeviceInfo.FirmwareVersion.c_str()));
    emit printI(tr("Variant:                 %1").arg(DeviceInfo.Variant.c_str()));
    emit printI(tr("IsFileCamera:            %1").arg((DeviceInfo.IsFileCamera ? "Yes" : "No")));
    emit printI(tr("Status:                  %1").arg((DeviceInfo.Status.Attached ?
                                                       "Attached to PhoXi Control. " : "Not Attached to PhoXi Control." +
                                                       DeviceInfo.Status.Ready ? "Ready to connect" : "Occupied")));
}

Photoneo::Photoneo(QWidget* parent) :
    CustomDock(parent), ui(new Ui::Photoneo),
    captured_cloud(new ct::Cloud)
{
    ui->setupUi(this);

    connect(ui->btn_connect, &QPushButton::clicked, this, &Photoneo::connectDevice);
    connect(ui->btn_capture, &QPushButton::clicked, this, &Photoneo::captureDevice);
    connect(ui->btn_add, &QPushButton::clicked, this, &Photoneo::add);
    connect(ui->btn_reset, &QPushButton::clicked, this, &Photoneo::reset);

    m_grabber = new PhotoneoGrabber;
    // m_grabber->moveToThread(&m_thread);
    //connect(&m_thread, &QThread::finished, m_grabber, &QObject::deleteLater);
    // connect(this, &Photoneo::connectSignal, m_grabber, &PhotoneoGrabber::connectDevice);
    // connect(this, &Photoneo::disconnectSignal, m_grabber, &PhotoneoGrabber::disconnectDevice);
    // connect(this, &Photoneo::captureOnceSignal, m_grabber, &PhotoneoGrabber::captureOnce);
    connect(m_grabber, &PhotoneoGrabber::sendPointcloud, this, &Photoneo::getPointCloud);
    connect(m_grabber, &PhotoneoGrabber::printI, this, &Photoneo::printI);
    connect(m_grabber, &PhotoneoGrabber::printE, this, &Photoneo::printE);
    //m_thread.start();

}

Photoneo::~Photoneo()
{
    // m_thread.quit();
    // if (!m_thread.wait(3000))
    // {
    //     m_thread.terminate();
    //     m_thread.wait();
    // }
    delete ui;
}

void Photoneo::connectDevice()
{
    if (m_grabber->isConnected())
    {
        if (!m_grabber->disconnectDevice())
        {
            printE("Disconnect to device failed");
            return;
        }
        ui->btn_connect->setIcon(QIcon(":/res/icon/connect.svg"));
        printI("Disconnect to device successfully!");
    }
    else
    {
        if (!m_grabber->connectDevice())
        {
            printE("Connect to device failed");
            return;
        }
        ui->btn_connect->setIcon(QIcon(":/res/icon/disconnect.svg"));
        printI("Connect to device successfully!");
    }
}

void Photoneo::captureDevice()
{
    if (!m_grabber->isConnected())
    {
        printE("Your device is not connected");
        return;
    }
    // if (m_grabber->isCapturing())
    // {
    //     printE("Your device is capturing");
    //     return;
    // }
    if (!m_grabber->captureOnce())
    {
        printE("Your device capture failed");
        return;
    }
    printI("Your device capture successfully!");
}

void Photoneo::add()
{
    if (captured_cloud->empty())
    {
        printW("Please capture a cloud first!");
        return;
    }
    m_cloudtree->appendCloud(captured_cloud, true);
    captured_cloud->clear();
    printI(QString("Add captured cloud[id:%1] done.").arg(captured_cloud->id()));
}

void Photoneo::reset()
{
    m_cloudview->removePointCloud(PHOTONEO_CAP_FLAG);
    // m_grabber->disconnect();
    // ui->btn_connect->setIcon(QIcon(":/res/icon/connect.svg"));
}

void Photoneo::getPointCloud(const ct::Cloud::Ptr& cloud, float time)
{
    printI(QString("capture cloud[id:%1] done, take time %2 ms.").arg(cloud->id()).arg(time));
    m_cloudview->addPointCloud(cloud);
    captured_cloud = cloud;
}
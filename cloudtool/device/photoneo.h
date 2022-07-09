/**
 * @file photoneo.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef CT_DEVICE_PHOTONEO_H
#define CT_DEVICE_PHOTONEO_H

#include "base/customdock.h"
#include <QThread>

#define PHOXI_PCL_SUPPORT
#include "PhoXi.h"


class PhotoneoGrabber : public QObject
{
    Q_OBJECT
public:
    explicit PhotoneoGrabber(QObject* parent = nullptr){}
    ~PhotoneoGrabber();
    bool connectDevice();
    bool disconnectDevice();
    bool captureOnce();
    bool isCapturing();
    bool isConnected();

signals:
    void sendPointcloud(const ct::Cloud::Ptr&, float);
    void printI(const QString&);
    void printE(const QString&);

private:
    void printDeviceInfoList(const std::vector<pho::api::PhoXiDeviceInformation> &);
    void printDeviceInfo(const pho::api::PhoXiDeviceInformation &);

    pho::api::PPhoXi m_device;
};

namespace Ui
{
    class Photoneo;
}

class Photoneo : public ct::CustomDock
{
    Q_OBJECT
public:
    explicit Photoneo(QWidget* parent = nullptr);
    ~Photoneo();

    void connectDevice();
    void captureDevice();
    void add();
    void reset();

signals:
    void connectSignal();
    void disconnectSignal();
    void captureOnceSignal();
    void captureContinueSignal();

public slots:
    void getPointCloud(const ct::Cloud::Ptr&, float);

private:
    Ui::Photoneo* ui;
    PhotoneoGrabber *m_grabber;
    QThread m_thread;
    ct::Cloud::Ptr captured_cloud;
};

#endif // CT_DEVICE_PHOTONEO_H

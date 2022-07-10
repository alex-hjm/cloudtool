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

    void searchDevice();
    void connectDevice();
    void captureDevice();
    void add();
    void reset();

public slots:
    void updateDeviceInfo(int);
    void updatePointCloud(const pho::api::PFrame&, float);
    
private:
    Ui::Photoneo* ui;
    ct::Cloud::Ptr m_captured_cloud;
    pho::api::PPhoXi m_device;
    pho::api::PhoXiFactory m_factory;
    std::vector<pho::api::PhoXiDeviceInformation> m_device_list;
};

#endif // CT_DEVICE_PHOTONEO_H

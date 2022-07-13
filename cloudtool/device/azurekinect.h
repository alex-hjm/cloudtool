/**
 * @file azurekinect.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef CT_DEVICE_AZUREKINECT_H
#define CT_DEVICE_AZUREKINECT_H

#include "base/customdock.h"
#include <QThread>

#include "k4a/k4a.hpp"

namespace Ui
{
    class AzureKinect;
}

class AzureKinect : public ct::CustomDock
{
    Q_OBJECT
public:
    explicit AzureKinect(QWidget* parent = nullptr);
    ~AzureKinect();

    void searchDevice();
    void connectDevice();
    void captureDevice();
    void add();
    void reset();

public slots:
    void updateDeviceInfo(int);
    
private:
    Ui::AzureKinect* ui;
    k4a::device m_device;
    ct::Cloud::Ptr m_captured_cloud;
};

#endif // CT_DEVICE_AZUREKINECT_H

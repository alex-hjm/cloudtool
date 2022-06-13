#ifndef DEVICEKINECTDK_H
#define DEVICEKINECTDK_H

#include <QDockWidget>
#include <QTimer>
#include <k4a/k4a.hpp>
#include "common/cloudtree.h"
#include "common/imagetree.h"

namespace Ui {
class DeviceKinectDK;
}

class DeviceKinectDK : public QDockWidget
{
    Q_OBJECT
public:
    explicit DeviceKinectDK(QWidget *parent = nullptr);
    ~DeviceKinectDK();

    void init(Console* &co,CloudView* &cv,CloudTree* &ct,ImageTree *&it,ImageView *&iv);
    void connectDevice();
    void captureDevice();
    void add();

protected:
    void output();
    void  closeEvent(QCloseEvent *event);
private:
    Ui::DeviceKinectDK *ui;
    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    ImageTree *image_tree;
    ImageView *image_view;
    QTimer *capture_timer;
    bool is_connected;
    bool is_capturing;
    int output_type;
    k4a::device device;
    k4a::capture capture;
    k4a::calibration calibration;
    k4a::transformation transformation;
    k4a_device_configuration_t config;
    Image::Ptr cv_rgbImage;
    Image::Ptr cv_depth;
    Image::Ptr cv_irImage;
    Cloud::Ptr captured_cloud;
};

#endif // DEVICEKINECTDK_H

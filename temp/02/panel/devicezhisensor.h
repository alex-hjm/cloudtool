#ifndef DEVICEZHISENSOR_H
#define DEVICEZHISENSOR_H

#include <QWidget>
#include <QDebug>
#include <QTimer>

#define NOMINMAX
#include <dkam_discovery.h>
#include <dkam_gige_camera.h>
#include <dkam_gige_stream.h>

#include "src/common/cloudtree.h"
#include "src/common/imagetree.h"

namespace Ui {
class DeviceZhiSensor;
}

class DeviceZhiSensor : public QWidget
{
    Q_OBJECT

public:
    explicit DeviceZhiSensor(QWidget *parent = nullptr);
    ~DeviceZhiSensor();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;
    ImageTree *imageTree;
    GraphicsView *graphicsView;


    void init();
    void searchDevice();
    void connectDevice();
    void captureDevice();
    void add();

protected:
    void captureCloud();
    void captureImage(int index);
    void closeEvent(QCloseEvent *event);

private:
    Ui::DeviceZhiSensor *ui;

    QTimer *captureTimer;
    pcl::console::TicToc time;
    bool isConnected;
    bool isCapturing;
    int refreshTime;
    int outputType;
    int width, height;
    int width_rgb,height_rgb;

    CloudXYZRGBN::Ptr capturedCloud;

    Discovery discovery;
    std::vector<DiscoveryInfo> discovery_info;
    GigeCamera camera;
    GigeStream *pointgigestream;
    GigeStream *rgbgigestream ;
    GigeStream *graygigestream ;

    PhotoInfo* point_data;
    PhotoInfo* gray_data;
    PhotoInfo* rgb_data;
    float* pointcloud;
    cv::Mat cv_grayImage;
    cv::Mat cv_rgbImage;

};

#endif // DEVICEZHISENSOR_H

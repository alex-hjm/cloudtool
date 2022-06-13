#ifndef DEVICEKINECTDK_H
#define DEVICEKINECTDK_H

#include <QWidget>
#include <QFileInfo>
#include <QDateTime>
#include <QTimer>
#include <k4a/k4a.hpp>

#include "src/common/cloudtree.h"
#include "src/common/imagetree.h"
struct pixel
{
    uint8_t Blue;
    uint8_t Green;
    uint8_t Red;
    uint8_t Alpha;
};

using DepthPixelVisualizationFunction = pixel(const uint16_t &value, const uint16_t &min, const uint16_t &max);

namespace Ui {
class DeviceKinectDK;
}


class DeviceKinectDK : public QWidget
{
    Q_OBJECT

public:
    explicit DeviceKinectDK(QWidget *parent = nullptr);
    ~DeviceKinectDK();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;
    ImageTree *imageTree;
    GraphicsView *graphicsView;

    void init();
    void connectDevice();
    void captureDevice();
    void add();

protected:
    void  captureCloud();
    void  captureImage(int);
    void  closeEvent(QCloseEvent *event);

private:
    Ui::DeviceKinectDK *ui;

    std::pair<uint16_t, uint16_t> GetDepthModeRange(const k4a_depth_mode_t depthMode);
    std::pair<int, int> GetDepthDimensions(const k4a_depth_mode_t depthMode);
    std::pair<int, int> GetColorDimensions(const k4a_color_resolution_t resolution);
    std::pair<uint16_t, uint16_t> GetIrLevels(const k4a_depth_mode_t depthMode);
    void ColorizeDepthImage(const k4a::image& depthImage,DepthPixelVisualizationFunction visualizationFn,
                            std::pair<uint16_t, uint16_t> expectedValueRange,std::vector<pixel>* buffer);
    static pixel ColorizeBlueToRed(const uint16_t &depthPixel, const uint16_t &min, const uint16_t &max);
    static pixel ColorizeGreyscale(const uint16_t &value, const uint16_t &min, const uint16_t &max);


    QTimer *captureTimer;
    pcl::console::TicToc time;
    bool isConnected;
    bool isCapturing;
    int outputType;
    k4a::device device;
    k4a::capture capture;
    k4a::calibration calibration;
    k4a::transformation transformation;
    k4a_device_configuration_t config;

    CloudXYZRGBN::Ptr capturedCloud;
    k4a::image colorImage;
    k4a::image depthImage;
    k4a::image infraredImage;
    cv::Mat cv_rgbImage;
    cv::Mat cv_depth;
    cv::Mat cv_irImage;
    std::vector<pixel> depthTextureBuffer;
    std::vector<pixel> irTextureBuffer;
    uint8_t* colorTextureBuffer;
};

#endif // DEVICEKINECTDK_H

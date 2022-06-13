#ifndef DEVICEZHISENSOR_H
#define DEVICEZHISENSOR_H

#include <QDockWidget>
#include <QTimer>
#include <QFileDialog>
#include <QDesktopServices>
#include <Eigen/Core>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#define NOMINMAX
#include <dkam_discovery.h>
#include <dkam_gige_camera.h>
#include <dkam_gige_stream.h>
#include "common/cloudtree.h"
#include "common/imagetree.h"
namespace Ui {
class DeviceZhisensor;
}

class DeviceZhisensor : public QDockWidget
{
    Q_OBJECT
public:
    explicit DeviceZhisensor(QWidget *parent = nullptr);
    ~DeviceZhisensor();

    void init(Console* &co,CloudView* &cv,CloudTree* &ct,ImageTree *&it,ImageView *&iv);
    void searchDevice();
    void connectDevice();
    void captureDevice();
    void getCamIntrinsics();
    void add();

protected:
    void output();
    void closeEvent(QCloseEvent *event);

private:
    Ui::DeviceZhisensor *ui;

    Console *console;
    CloudView *cloud_view;
    CloudTree *cloud_tree;
    ImageTree *image_tree;
    ImageView *image_view;

    QTimer *capture_timer;
    bool is_connected;
    bool is_capturing;
    int output_type;
    int width, height;
    int width_rgb,height_rgb;

    std::vector<DiscoveryInfo> discovery_info;
    GigeCamera camera;
    GigeStream *pointgigestream;
    GigeStream *rgbgigestream ;
    GigeStream *graygigestream ;

    Image::Ptr cv_rgbImage;
    Image::Ptr cv_grayImage;
    Cloud::Ptr captured_cloud;
};

#endif // DEVICEZHISENSOR_H

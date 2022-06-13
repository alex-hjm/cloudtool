#ifndef RANGEIMAGE_H
#define RANGEIMAGE_H

#include <QWidget>

#include <pcl/visualization/common/float_image_utils.h>
#include <vtkImageActor.h>

#include "src/common/cloudtree.h"

#include "src/modules/keypointsmodule.h"

namespace Ui {
class RangeImage;
}

class RangeImage : public QWidget
{
    Q_OBJECT

public:
    explicit RangeImage(QWidget *parent = nullptr);
    ~RangeImage();

    Console *console;
    CloudView *cloudView;
    CloudTree *cloudTree;

     void init();
     void capture();

protected:
    void closeEvent(QCloseEvent *event);

public slots:
    void updateRangeImage(Eigen::Affine3f&);
private:
    Ui::RangeImage *ui;

    float angularResolution;
    float noiseLevel;
    float minRange;
    int borderSize;

    float* floatImage;
    unsigned char* rgbImage;
    void* data;

    vtkSmartPointer<vtkImageData> image;
    vtkSmartPointer<vtkImageActor> actor;
    vtkSmartPointer<vtkRenderer> render;

    boost::shared_ptr<pcl::RangeImage> rangeImagePtr;
    pcl::RangeImage::CoordinateFrame coordinateFrame ;
    pcl::PointCloud<pcl::PointWithViewpoint> farRanges;

    Cloud selectedCloud;
};

#endif // RANGEIMAGE_H

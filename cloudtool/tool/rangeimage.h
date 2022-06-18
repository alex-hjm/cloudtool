/**
 * @file rangeimage.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#ifndef CT_TOOL_RANGEIMAGE_HS
#define CT_TOOL_RANGEIMAGE_HS

#include "base/customdialog.h"

#include "modules/keypoints.h"

#include <pcl/range_image/range_image.h>
#include <vtkImageData.h>
#include <vtkImageActor.h>

namespace Ui
{
    class RangeImage;
}

class RangeImage : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit RangeImage(QWidget* parent = nullptr);
    ~RangeImage();

    virtual void init();
    virtual void reset();

public slots:
    void updateRangeImage(Eigen::Affine3f&);

private:
    Ui::RangeImage* ui;
    ct::RangeImage::Ptr m_range_image;
    vtkSmartPointer<vtkImageData> m_image_data;
    vtkSmartPointer<vtkRenderer> m_ren;
    vtkSmartPointer<vtkImageActor> m_actor;
};

#endif  // CT_TOOL_RANGEIMAGE_HS

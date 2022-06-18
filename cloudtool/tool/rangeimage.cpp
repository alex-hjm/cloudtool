/**
 * @file rangeimage.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-15
 */
#include "rangeimage.h"

#include "ui_rangeimage.h"

#include <pcl/visualization/common/float_image_utils.h>

#define RANGE_IMAGE_FLAG  "range_image"

RangeImage::RangeImage(QWidget* parent) :
    CustomDialog(parent), ui(new Ui::RangeImage),
    m_range_image(new ct::RangeImage),
    m_image_data(vtkSmartPointer<vtkImageData>::New()),
    m_ren(vtkSmartPointer<vtkRenderer>::New()),
    m_actor(vtkSmartPointer<vtkImageActor>::New())
{
    ui->setupUi(this);
    ui->qvtkwidget->GetRenderWindow()->AddRenderer(m_ren);
    m_ren->SetBackground(1, 1, 1);
    this->resize(64, 48);
}

RangeImage::~RangeImage() { delete ui; }

void RangeImage::init() { connect(m_cloudview, &ct::CloudView::viewerPose, this, &RangeImage::updateRangeImage); }

void RangeImage::updateRangeImage(Eigen::Affine3f& viewer_pose)
{
    std::vector<ct::Cloud::Ptr> selected_clouds = m_cloudtree->getSelectedClouds();
    if (selected_clouds.empty())
    {
        m_image_data = vtkSmartPointer<vtkImageData>::New();
        m_ren->RemoveActor(m_actor);
        ui->qvtkwidget->GetRenderWindow()->Render();
        m_cloudview->removePointCloud(RANGE_IMAGE_FLAG);
        return;
    }
    ct::Cloud::Ptr cloud = selected_clouds.front();
    m_range_image->createFromPointCloud(*cloud, pcl::deg2rad(0.5f), pcl::deg2rad(360.0f), pcl::deg2rad(180.0f), viewer_pose);
    this->resize(3 * m_range_image->width, 3 * m_range_image->height);
    float* ranges_array = m_range_image->getRangesArray();
    unsigned char* rgbImage = pcl::visualization::FloatImageUtils::getVisualImage(ranges_array, m_range_image->width, m_range_image->height);
    void* data = const_cast<void*> (reinterpret_cast<const void*> (rgbImage));
    m_image_data->SetExtent(0, m_range_image->width - 1, 0, m_range_image->height - 1, 0, 0);
    m_image_data->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
    m_image_data->GetPointData()->GetScalars()->SetVoidArray(data, 3 * m_range_image->width * m_range_image->height, 1);
    delete[] ranges_array;
    delete[] rgbImage;
    m_actor->SetInputData(m_image_data);
    m_actor->Update();
    m_ren->AddActor(m_actor);
    m_ren->ResetCamera();
    ui->qvtkwidget->GetRenderWindow()->Render();
    m_cloudview->addPointCloudFromRangeImage(m_range_image, RANGE_IMAGE_FLAG, QColorConstants::Green);
    m_cloudview->setPointCloudSize(RANGE_IMAGE_FLAG, cloud->pointSize() + 2);
}

void RangeImage::reset()
{
    m_cloudview->removePointCloud(RANGE_IMAGE_FLAG);
    disconnect(m_cloudview, &ct::CloudView::viewerPose, this, &RangeImage::updateRangeImage);
}

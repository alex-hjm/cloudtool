/**
 * @file cloudview.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-28
 */
#include "cloudview.h"

CT_BEGIN_NAMESPACE

CloudView::CloudView(QWidget *parent)
  : QVTKOpenGLNativeWidget(parent), m_render(vtkSmartPointer<vtkRenderer>::New()),
    m_renderwindow(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New())
{
    m_renderwindow->AddRenderer(m_render);
    m_viewer.reset(new pcl::visualization::PCLVisualizer(m_render, m_renderwindow, "viewer", false));
    this->setRenderWindow(m_viewer->getRenderWindow());
    m_viewer->setupInteractor(this->interactor(), this->renderWindow());
    m_viewer->setBackgroundColor((double)243.0/255.0, (double)243.0/255.0, (double)243.0/255.0);
    m_renderwindow->Render();
}

void CloudView::addCloud(const Cloud::Ptr &cloud)
{
    std::string const id(cloud->id().toStdString());
    if (!m_viewer->contains(id)) {
        m_viewer->addPointCloud<PointXYZRGBN>(cloud, id);
        m_viewer->resetCamera();
    } else {
        pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN> rgb(cloud);
        m_viewer->updatePointCloud<PointXYZRGBN>(cloud, rgb, id);
    }
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud->pointSize(), id);
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, cloud->opacity(), id);
    m_renderwindow->Render();
}

void CloudView::addCloudBBox(const Cloud::Ptr &cloud)
{
    std::string const id(cloud->bboxId().toStdString());
    if (!m_viewer->contains(id)) {
        m_viewer->addCube(cloud->bbox().translation, cloud->bbox().rotation, cloud->bbox().width, cloud->bbox().height,
                          cloud->bbox().depth, id);
    } else {
        m_viewer->removeShape(id);
        m_viewer->addCube(cloud->bbox().translation, cloud->bbox().rotation, cloud->bbox().width, cloud->bbox().height,
                          cloud->bbox().depth, id);
    }
    m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                          pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, id);
    m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, cloud->bboxColor().rf(),
                                          cloud->bboxColor().gf(), cloud->bboxColor().bf(), id);
    m_renderwindow->Render();
}


void CloudView::removeCloud(const QString &id)
{
    m_viewer->removePointCloud(id.toStdString());
    m_renderwindow->Render();
}

void CloudView::removeCloudBBox(const QString &id)
{
    m_viewer->removeShape(id.toStdString());
    m_renderwindow->Render();
}

void CloudView::removeAllClouds()
{
    m_viewer->removeAllPointClouds();
    m_renderwindow->Render();
}

void CloudView::setCloudSize(const QString &id, int size)
{
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id.toStdString());
    m_renderwindow->Render();
}

void CloudView::setCloudOpacity(const QString &id, float opacity)
{
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, id.toStdString());
    m_renderwindow->Render();
}

bool CloudView::contains(const QString& id)
{
    return m_viewer->contains(id.toStdString());
}

void CloudView::resetCamera()
{
    m_viewer->resetCamera();
    m_renderwindow->Render();
}

CT_END_NAMESPACE
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
    m_renderwindow->Render();
}

void CloudView::addCloud(const Cloud::Ptr &cloud)
{
    if (!m_viewer->contains(cloud->id)) {
        m_viewer->addPointCloud<PointXYZRGBN>(cloud, cloud->id);
    } else {
        pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN> rgb(cloud);
        m_viewer->updatePointCloud<PointXYZRGBN>(cloud, rgb, cloud->id);
    }
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud->point_size, cloud->id);
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, cloud->opacity, cloud->id);
    m_renderwindow->Render();
}

void CloudView::addCloudBox(const Cloud::Ptr &cloud)
{
    if (!m_viewer->contains(cloud->bbox_id)) {
        m_viewer->addCube(cloud->bbox.translation, cloud->bbox.rotation, cloud->bbox.width, cloud->bbox.height,
                          cloud->bbox.depth, cloud->bbox_id);
    } else {
        m_viewer->removeShape(cloud->bbox_id);
        m_viewer->addCube(cloud->bbox.translation, cloud->bbox.rotation, cloud->bbox.width, cloud->bbox.height,
                          cloud->bbox.depth, cloud->bbox_id);
    }
    m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                          pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cloud->bbox_id);
    m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, cloud->bbox_rgb.rf(),
                                          cloud->bbox_rgb.gf(), cloud->bbox_rgb.bf(), cloud->bbox_id);
    m_renderwindow->Render();
}

void CloudView::addCloudNormals(const Cloud::Ptr &cloud, int level, float scale)
{
    if (!m_viewer->contains(cloud->normals_id)) {
        m_viewer->addPointCloudNormals<PointXYZRGBN>(cloud, level, scale, cloud->normals_id);
    } else {
        m_viewer->removePointCloud(cloud->normals_id);
        m_viewer->addPointCloudNormals<PointXYZRGBN>(cloud, level, scale, cloud->normals_id);
    }
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, cloud->normals_rgb.rf(),
                                               cloud->normals_rgb.gf(), cloud->normals_rgb.bf(), cloud->normals_id);
    m_renderwindow->Render();
}

void CloudView::removeCloud(const std::string &id)
{
    m_viewer->removePointCloud(id);
    m_renderwindow->Render();
}

void CloudView::removeCloudBox(const std::string &id)
{
    m_viewer->removeShape(id);
    m_renderwindow->Render();
}

void CloudView::removeShape(const std::string &id)
{
    m_viewer->removeShape(id);
    m_renderwindow->Render();
}

void CloudView::removeAllClouds()
{
    m_viewer->removeAllPointClouds();
    m_renderwindow->Render();
}

void CloudView::removeAllShapes()
{
    m_viewer->removeAllShapes();
    m_renderwindow->Render();
}

void CloudView::setCloudColor(const Cloud::Ptr &cloud, const RGB &rgb)
{
    pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBN> color(cloud, rgb.r, rgb.g, rgb.b);
    m_viewer->updatePointCloud(cloud, color, cloud->id);
    m_renderwindow->Render();
}

void CloudView::setCloudColor(const Cloud::Ptr &cloud, const std::string &axis)
{
    pcl::visualization::PointCloudColorHandlerGenericField<PointXYZRGBN> field_color(cloud, axis);
    m_viewer->updatePointCloud(cloud, field_color, cloud->id);
    m_renderwindow->Render();
}

void CloudView::setCloudColor(const std::string &id, const RGB &rgb)
{
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rgb.rf(), rgb.gf(), rgb.bf(), id);
    m_renderwindow->Render();
}

void CloudView::setCloudSize(const std::string &id, float size)
{
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id);
    m_renderwindow->Render();
}

void CloudView::setShapeColor(const std::string &id, const RGB &rgb)
{
    m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rgb.rf(), rgb.gf(), rgb.bf(), id);
    m_renderwindow->Render();
}

bool CloudView::contains(const std::string& id)
{
    return m_viewer->contains(id);
}

CT_END_NAMESPACE
/**
 * @file cloudview.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-28
 */
#include "cloudview.h"

#include <vtkAxesActor.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkCaptionActor2D.h>

CT_BEGIN_NAMESPACE

CloudView::CloudView(QWidget *parent): QVTKOpenGLNativeWidget(parent), 
    m_render(vtkSmartPointer<vtkRenderer>::New()),
    m_renderwindow(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New()),
    m_axes(vtkSmartPointer<vtkOrientationMarkerWidget>::New()),
    m_update_fps(vtkSmartPointer<FPSCallback>::New()),
    m_show_fps(true),
    m_show_axes(true)
{
    m_renderwindow->AddRenderer(m_render);
    this->setRenderWindow(m_renderwindow);
    m_viewer.reset(new pcl::visualization::PCLVisualizer(m_render, m_renderwindow, "viewer", false));
    m_viewer->setupInteractor(this->interactor(), this->renderWindow());
    m_viewer->setShowFPS(false);
    
    // axes
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetVisibility(m_show_axes);
    axes->GetXAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->SetColor(0, 0, 0); 
    axes->GetYAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->SetColor(0, 0, 0); 
    axes->GetZAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->SetColor(0, 0, 0); 
    axes->GetXAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->SetBold(0); 
    axes->GetYAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->SetBold(0); 
    axes->GetZAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->SetBold(0); 
    axes->GetXAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->SetShadow(0);
    axes->GetYAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->SetShadow(0);
    axes->GetZAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->SetShadow(0);
    m_axes->SetOrientationMarker(axes);
    m_axes->SetInteractor(this->interactor());
    m_axes->SetViewport(0.92, 0, 1, 0.08);
    m_axes->SetEnabled(true);
    m_axes->InteractiveOff();
    
    // fps
    m_render->AddObserver (vtkCommand::EndEvent, m_update_fps);
    vtkSmartPointer<vtkTextActor> txt = vtkSmartPointer<vtkTextActor>::New();
    txt->GetTextProperty()->SetColor(0, 0, 0); 
    txt->SetPosition(0, 20);  
    txt->SetVisibility(m_show_fps);
    txt->SetInput ("0 FPS");
    m_update_fps->actor = txt;
    m_render->AddActor (txt);

    // render
    m_renderwindow->Render();
    setContextMenuPolicy(Qt::CustomContextMenu);
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

void CloudView::setBackgroundColor(const RGB& rgb)
{
    m_viewer->setBackgroundColor(rgb.rf(), rgb.gf(),  rgb.bf());
    m_renderwindow->Render();
}

void CloudView::showFPS(bool enable)
{
    m_update_fps->actor->SetVisibility(enable);
    m_show_fps = enable;
    m_renderwindow->Render();
}

void CloudView::setFPSColor(const RGB& rgb)
{
    m_update_fps->actor->GetTextProperty()->SetColor(rgb.rf(), rgb.gf(),  rgb.bf());
    m_renderwindow->Render();
}

void CloudView::showAxes(bool enable)
{
    m_axes->GetOrientationMarker()->SetVisibility(enable);
    m_show_axes = enable;
    m_renderwindow->Render();
}

void CloudView::setAxesColor(const RGB& rgb)
{
    vtkSmartPointer<vtkAxesActor> axes = vtkAxesActor::SafeDownCast(m_axes->GetOrientationMarker());
    axes->GetXAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->SetColor(rgb.rf(), rgb.gf(),  rgb.bf()); 
    axes->GetYAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->SetColor(rgb.rf(), rgb.gf(),  rgb.bf()); 
    axes->GetZAxisCaptionActor2D()->GetTextActor()->GetTextProperty()->SetColor(rgb.rf(), rgb.gf(),  rgb.bf()); 
    m_renderwindow->Render();
}

void CloudView::saveScreenshot(const QString& file)
{
    m_viewer->saveScreenshot(file.toLocal8Bit().toStdString());
}

void CloudView::saveCameraParam(const QString& file)
{
    m_viewer->saveCameraParameters(file.toLocal8Bit().toStdString());
}

void CloudView::loadCameraParam(const QString& file)
{
    m_viewer->loadCameraParameters(file.toLocal8Bit().toStdString());
    m_renderwindow->Render();
}

void CloudView::FPSCallback::Execute (vtkObject* caller, unsigned long, void*)
{
  auto *ren = reinterpret_cast<vtkRenderer *> (caller);
  last_fps = 1.0f / static_cast<float> (ren->GetLastRenderTimeInSeconds ());
  char buf[128];
  sprintf (buf, "%.1f FPS", last_fps);
  actor->SetInput (buf);
}

CT_END_NAMESPACE
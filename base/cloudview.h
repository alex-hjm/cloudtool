/**
 * @file cloudview.h
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-01-28
 */
#ifndef __BASE_CLOUDVIEW_H__
#define __BASE_CLOUDVIEW_H__

#include "cloud.h"

#include <QVTKOpenGLNativeWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkOrientationMarkerWidget.h>

CT_BEGIN_NAMESPACE

class CT_EXPORT CloudView : public QVTKOpenGLNativeWidget
{
    Q_OBJECT
public:
    explicit CloudView(QWidget *parent = nullptr);

    void addCloud(const Cloud::Ptr &cloud);
    void addCloudBBox(const Cloud::Ptr &cloud);

    void removeCloud(const QString &id);
    void removeCloudBBox(const QString &id);
    void removeAllClouds();

    void setCloudSize(const QString &id, int size);
    void setCloudOpacity(const QString &id, float opacity);

    bool contains(const QString& id);
    void resetCamera();

private:
    pcl::visualization::PCLVisualizer::Ptr m_viewer;
    vtkSmartPointer<vtkRenderer> m_render;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderwindow;
};

CT_END_NAMESPACE
#endif // __BASE_CLOUDVIEW_H__

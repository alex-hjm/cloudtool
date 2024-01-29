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
    void addCloudBox(const Cloud::Ptr &cloud);
    void addCloudNormals(const Cloud::Ptr &cloud, int level, float scale);

    void removeCloud(const std::string &id);
    void removeCloudBox(const std::string &id);
    void removeShape(const std::string &id);
    void removeAllClouds();
    void removeAllShapes();

    void setCloudColor(const Cloud::Ptr &cloud, const RGB &rgb);
    void setCloudColor(const std::string &id, const RGB &rgb);
    void setCloudColor(const Cloud::Ptr &cloud, const std::string &axis);
    void setCloudSize(const std::string &id, float size);
    void setShapeColor(const std::string &id, const RGB &rgb);

    bool contains(const std::string& id);

private:
    pcl::visualization::PCLVisualizer::Ptr m_viewer;
    vtkSmartPointer<vtkRenderer> m_render;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderwindow;
};

CT_END_NAMESPACE
#endif // __BASE_CLOUDVIEW_H__

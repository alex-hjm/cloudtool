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

#include <QMenu>
#include <QSettings>

CT_BEGIN_NAMESPACE

struct PointPickInfo {
    QString id;
    double   x;
    double   y;
    double   z;
};

class CT_EXPORT CloudView : public QVTKOpenGLNativeWidget
{
    Q_OBJECT
public:
    explicit CloudView(QWidget *parent = nullptr);

    void addCloud(const Cloud::Ptr &cloud);
    void addCloudBBox(const Cloud::Ptr &cloud);
    void addCoord(const Coord& coord, bool showId);

    void removeCloud(const QString &id);
    void removeCloudBBox(const QString &id);
    void removeAllClouds();
    void removeCoord(const QString& id);
    void removeCoordId(const QString& id);
    void removeAllCoords();

    void setCloudColor(const QString &id, const RGB& rgb);
    void setCloudColor(const Cloud::Ptr& cloud, const RGB& rgb);
    void setShapeColor(const QString &id, const RGB& rgb);
    void setCloudSize(const QString &id, int size);
    void setCloudOpacity(const QString &id, float opacity);
    void resetCloudColor(const Cloud::Ptr& cloud);

    bool contains(const QString& id);
    void resetCamera();
    void setBackgroundColor(const RGB& rgb);

    void showFPS(bool enable);
    bool FPSEnable() {return m_show_fps; }
    void setFPSColor(const RGB& rgb);
    void showAxes(bool enable);
    bool AxesEnable() {return m_show_axes; }
    void setAxesColor(const RGB& rgb);
    
    void saveScreenshot(const QString& file);
    void saveCameraParam(const QString& file);
    void loadCameraParam(const QString& file);

protected:
    void mousePressEvent(QMouseEvent* event);

signals:
    void pointPickEvent(const PointPickInfo&);

private:
    struct CT_EXPORT FPSCallback : public vtkCommand {
        static FPSCallback *New () { return (new FPSCallback); }

        FPSCallback () = default;
        FPSCallback (const FPSCallback& src)  = default;
        void Execute (vtkObject*, unsigned long event_id, void*) override;

        vtkTextActor *actor{nullptr};
        bool decimated{false};
        float last_fps{0.0f};
    };

    pcl::visualization::PCLVisualizer::Ptr m_viewer;
    vtkSmartPointer<vtkRenderer> m_render;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderwindow;
    vtkSmartPointer<vtkOrientationMarkerWidget> m_axes;
    vtkSmartPointer<FPSCallback> m_update_fps;

    bool m_show_fps;
    bool m_show_axes;
};

CT_END_NAMESPACE
#endif // __BASE_CLOUDVIEW_H__

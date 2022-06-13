#ifndef CLOUDVIEW_H
#define CLOUDVIEW_H
#include <QWidget>
#include <QMenu>
#include <QResizeEvent>
#include <QMouseEvent>
#include <QMimeData>
#include <QUrl>
#include <QDropEvent>
#include <QDebug>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2)
VTK_MODULE_INIT(vtkRenderingFreeType)

#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <vtkDataSet.h>
#include <vtkPolyData.h>
#include <vtkPointPicker.h>
#include <vtkAreaPicker.h>
#include <vtkOpenGLRenderer.h>
#include <vtkRenderer.h>
#include <vtkExtractGeometry.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPlanes.h>
#include <vtkCamera.h>


#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/common/common.h>

#include "customtype.h"
namespace{
    class DisableInteractorStyle : public vtkInteractorStyleTrackballCamera
    {
    public:
        static DisableInteractorStyle* New();
        vtkTypeMacro(DisableInteractorStyle, vtkInteractorStyleTrackballCamera);

        virtual void OnLeftButtonDown() override{}
        virtual void OnMiddleButtonDown() override{}
        virtual void OnRightButtonDown() override{}
        virtual void OnMouseWheelForward() override{}
        virtual void OnMouseWheelBackward() override{}
    };
    vtkStandardNewMacro(DisableInteractorStyle);
}

struct Point2D{
    Point2D(){}
    Point2D(int x,int y):x(x),y(y){}
    float x;
    float y;
};

class CloudView:public QVTKOpenGLNativeWidget
{
    Q_OBJECT
public:

    explicit CloudView(QWidget *parent = nullptr);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void init();

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //update cloud
    void updateCloud(const CloudXYZRGBN::Ptr &cloud,const string &id);
    void updateCloud(const CloudXYZRGB::Ptr &cloud,const string &id);
    void updateCloud(const CloudXYZI::Ptr &cloud,const string &id);
    void updateCloud(const CloudXYZL::Ptr &cloud,const string &id);
    void updateCloud(const CloudXYZRGBA::Ptr &cloud,const string &id);
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //update correspondence
    void updateCorrespondences(const CloudXYZRGBN::Ptr &source, const CloudXYZRGBN::Ptr &target,
                               const CorrespondencesPtr &correspondences, const std::string &id);
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //update mesh
    void updateMesh(const PolygonMesh::Ptr &polymesh, const string &id);
    void updateMeshLine(const PolygonMesh::Ptr &polymesh, const string &id);
    void updatePolygon(const PolyXYZRGBN &polygon,double r,double g,double b,const string &id);
    void updatePolyLine(const CloudXYZRGBN::Ptr &cloud,double r,double g,double b,const string &id);
    void updateNormols(const CloudXYZRGBN::Ptr &cloud,int level,float scale,const string &id);
    //update model
    void updateModel(const Model &model);

    //update shape
    //2d->3d(display to world)
    PointXYZRGBN displayToWorld(const Point2D &pos);
    Point2D worldToDisplay(const PointXYZRGBN& point);
    void updateLine(const Point2D &start,const Point2D &end,double r,double g,double b,const string &id );
    void updatePolyLine(const std::vector<Point2D> &points, double r, double g, double b,const string &id);
    void updateArrow(const Point2D &start,const Point2D &end,double r,double g,double b,const string &id );
    void updateText(const Point2D &start,const string &text,double r, double g, double b,double fontsize,const string &id);
    //3d
    void updateCube(const Eigen::Vector3f &translation,const Eigen::Quaternionf &rotation,double width,double height,double depth,const string id);
    void updateCube(const PointXYZRGBN &min,PointXYZRGBN &max,double r,double g,double b,const string id);
    void updateCoord(double scale,const Eigen::Affine3f&affine,const string&id);
    void updateCoord(double scale,const string&id);
    void updateCoord(double scale,const Eigen::Vector3f &center,const string&id);
    void updateLine(const PointXYZRGBN &pt1,const PointXYZRGBN &pt2,double r,double g,double b,const string &id );
    void updateSphere(const PointXYZRGBN &center, double radius, double r, double g, double b, const string &id);
    void updateArrow(const PointXYZRGBN &pt1, const PointXYZRGBN &pt2, double r, double g, double b, bool display_length, const string &id);
    void updateText3D(const string &text,const PointXYZRGBN&position,double scale,double r,double g, double b,const string &id);
    void updateBoundingBox(const BoundingBox &boundingbox,const string &id);

    //point pick
    int singlePick(Point2D &p);
    std::vector<int> areaPick(const std::vector<Point2D>&points,const CloudXYZRGBN::Ptr &cloud,bool type);

    //update pose
    void updateCloudPose(const string& ,const Eigen::Affine3f &);
    void updateShapePose(const string& ,const Eigen::Affine3f &);
    void updateCoordPose(const string& ,const Eigen::Affine3f &);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //remove
    void removeCloud(const string &id);
    void removeMesh(const string &id);
    void removeShape(const string &id);
    void removeCoord(const string &id);
    void removeText3D (const string &id);
    void removeCorrespondences(const string &id);
    void removeAllClouds();
    void removeAllCoord();
    void removeAllShapes();

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //set cloud properties
    void setCloudColor(const CloudXYZRGBN::Ptr &cloud,const string &id,double r, double g, double b);
    void setCloudColor(const CloudXYZRGBN::Ptr &cloud,const string &id,string axis);
    void resetCloudColor(const CloudXYZRGBN::Ptr &cloud,const string &id);
    void setCloudSize(const string &id,float size);
    void setCloudOpacity(const string &id,float value);

    //set background properties
    void setBackgroundColor(double r, double g, double b);
    void resetBackgroundColor();

    //set shape properties
    void setSurfaceForAllShapes();
    void setPointsForAllShapes();
    void setWireframeForAllShapes();
    void setShapeColor(const string &shapeid,double r, double g, double b);
    void setShapeSize(const string &shapeid,float size);
    void setShapeOpacity(const string &shapeid,float value);
    void setShapeLineWidth(const string &shapeid,float value);
    void setShapeFontSize(const string &shapeid,float value);
    void setShapeRepersentation(const string &shapeid,int type);
    void setShapeShading(const string &shapeid,int type);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //camera
    void updateCamera();
    void resetCamera();
    void resetCameraViewpoint (const std::string &id);
    Eigen::Affine3f getViewerPose();
    void setViewerPose(const Eigen::Affine3f& viewer_pose);
    void initCameraParameters();
    void saveCameraParameters (const std::string &file);
    void setCameraParameters(const Eigen::Matrix3f &intrinsics, const Eigen::Matrix4f &extrinsics);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //display
    void showInfoText(const string &text,int height,const string &id);
    void showCloudId(const string &id);
    void showAxes();

    void setShowCloudId(const bool &enable);
    void setShowFPS(const bool &enable);
    void setshowAxes(const bool &enable);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //interactor
    void setInteractorEnable(const bool &enable);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //other
    bool contains(const string &id);
    void saveScreenshot(const std::string &file);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //viewport
    void setTopview();
    void setFrontview();
    void setLeftSideview();
    void setBackview();
    void setRightSideview();
    void setBottomview();

protected:
    void resizeEvent(QResizeEvent* size);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void dragEnterEvent(QDragEnterEvent *event);
    void dropEvent(QDropEvent *event);

private:
    bool showCloudIdEnable;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    //std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    vtkSmartPointer<vtkRenderer> m_ren;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renWnd;
    vtkSmartPointer<vtkOrientationMarkerWidget> axes;
    vtkSmartPointer<vtkPointPicker> point_picker;

signals:
    void dropFilePath(QStringList);
    void viewerPose(Eigen::Affine3f);
    void sizeChanged(QSize);
    void mouseLeftPressPos(Point2D);
    void mouseLeftReleasePos(Point2D);
    void mouseRightPressPos(Point2D);
    void mouseRightReleasePos(Point2D);
    void mouseMovePos(Point2D);
    void pointPicked(Point2D);
    void areaPicked(std::vector<int>);
};

#endif // CLOUDVIEW_H

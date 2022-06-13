#ifndef CLOUDVIEW_H
#define CLOUDVIEW_H
#include <QMimeData>
#include <QUrl>
#include <QDropEvent>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2)
VTK_MODULE_INIT(vtkRenderingFreeType)
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkPointPicker.h>
#include <pcl/geometry/planar_polygon.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "cloud.h"
typedef pcl::PlanarPolygon<PointXYZRGBN> PolyXYZRGBN;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> PrincipalCurvatures;
typedef pcl::CorrespondencesPtr CorrespondencesPtr;
typedef pcl::PolygonMesh PolygonMesh;
typedef pcl::ModelCoefficients ModelCoefficients;

struct Point2D{
    Point2D(){}
    Point2D(int x,int y):x(x),y(y){}
    bool operator !=(Point2D &pt){
        if(this->x!=pt.x ||this->y!=pt.y)
            return true;
        else
            return false;
    }
    bool operator ==(Point2D &pt){
        if(this->x==pt.x && this->y==pt.y)
            return true;
        else
            return false;
    }
    float x=0.0f;
    float y=0.0f;
};

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

class CloudView:public QVTKOpenGLNativeWidget
{
    Q_OBJECT
public:

    explicit CloudView(QWidget *parent = nullptr);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void init();
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //add
    void addCloud (const Cloud::Ptr &cloud,const std::string &id);
    void addCoord (double scale,const std::string &id);
    void addCoord (double scale,const Eigen::Affine3f &affine,const std::string &id);
    void addCoord (double scale, float x, float y, float z, const std::string &id);
    void addText (const std::string &text,int xpos, int ypos,const std::string &id);
    void addText (const std::string &text, int xpos, int ypos, double r, double g, double b,
                  const std::string &id );
    void addText (const std::string &text, int xpos, int ypos, int fontsize, double r, double g, double b,
                  const std::string &id);
    void addText3D (const std::string &text,const PointXYZRGBN &position,double textScale,double r, double g, double b,
                    const std::string &id);
    void addCloudNormals (const Cloud::Ptr &cloud,int level, float scale,const std::string &id);
    void addCloudNormals (const Cloud::Ptr &cloud,const CloudNormal::Ptr &normals,int level, float scale,
                          const std::string &id);
    void addCloudPrincipalCurvatures (const Cloud::Ptr &cloud,const PrincipalCurvatures::Ptr &pcs,
                                      int level, float scale,const std::string &id);
    void addCloudPrincipalCurvatures (const Cloud::Ptr &cloud,const CloudNormal::Ptr &normals,
                                      const PrincipalCurvatures::Ptr &pcs,int level, float scale,const std::string &id);
    void addPolygonMesh (const PolygonMesh::Ptr &polymesh,const std::string &id);
    void addPolylineFromPolygonMesh (const PolygonMesh::Ptr &polymesh,const std::string &id);
    void addCorrespondences (const Cloud::Ptr &source_points,const Cloud::Ptr &target_points,
                             const CorrespondencesPtr &correspondences,const std::string &id);
    void addPolygon (const Cloud::Ptr &cloud,const std::string &id);
    void addPolygon (const Cloud::Ptr &cloud,double r, double g, double b,const std::string &id);
    void addPolygon (const PolyXYZRGBN::Ptr&polygon,double r, double g, double b,const std::string &id);
    void addLine (const PointXYZRGBN &pt1,const PointXYZRGBN &pt2,const std::string &id);
    void addLine (const PointXYZRGBN &pt1,const PointXYZRGBN &pt2,double r, double g, double b,const std::string &id);
    void addArrow (const PointXYZRGBN &pt1,const PointXYZRGBN &pt2,double r, double g, double b,
                   bool display_length,const std::string &id);
    void addSphere (const PointXYZRGBN &center, double radius, const std::string &id);
    void addSphere (const PointXYZRGBN &center, double radius,double r, double g, double b, const std::string &id);
    void addCylinder (const ModelCoefficients::Ptr &coefficients,const std::string &id);
    void addSphere (const ModelCoefficients::Ptr &coefficients, const std::string &id);
    void addLine (const ModelCoefficients::Ptr &coefficients, const std::string &id);
    void addPlane (const ModelCoefficients::Ptr &coefficients, const std::string &id);
    void addPlane (const ModelCoefficients::Ptr &coefficients, double x, double y, double z,const std::string &id);
    void addCircle (const ModelCoefficients::Ptr &coefficients, const std::string &id);
    void addCone (const ModelCoefficients::Ptr &coefficients, const std::string &id);
    void addCube (const ModelCoefficients::Ptr &coefficients, const std::string &id);
    void addCube (const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation,double width, double height, double depth,
                  const std::string &id);
    void addCube(const PointXYZRGBN &min,PointXYZRGBN &max,double r,double g,double b,const std::string id);
    void addCube(const Box &box,const std::string &id);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //2d->3d(display to world)
    PointXYZRGBN displayToWorld(const Point2D &pos);
    Point2D worldToDisplay(const PointXYZRGBN& point);
    void addLine(const Point2D &start,const Point2D &end,double r,double g,double b,const std::string &id );
    void addPolyLine(const std::vector<Point2D> &points, double r, double g, double b,const std::string &id);
    void addArrow(const Point2D &start,const Point2D &end,double r,double g,double b,const std::string &id );
    void addText(const Point2D &start,const std::string &text,double r, double g, double b,double fontsize,const std::string &id);

    //point pick
    int singlePick(const Point2D &p);
    std::vector<int> areaPick(const std::vector<Point2D> &points, const Cloud::Ptr &cloud, bool type);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //update
    void updateCloud(const Cloud::Ptr &cloud,const std::string &id);
    inline void updateCoord(double scale,const std::string &id)
    { this->addCoord(scale,id);}
    inline void updateCoord(double scale,const Eigen::Affine3f &affine,const std::string &id)
    { this->addCoord(scale,affine,id);}
    inline void updateCube(const Box &box,const std::string &id)
    { this->addCube(box,id);}
    inline void updateCloudNormals(const Cloud::Ptr &cloud,int level, float scale,const std::string &id)
    { this->addCloudNormals(cloud,level,scale,id);}
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //update pose
    inline void updateShapePose(const std::string &id, const Eigen::Affine3f& pose)
    {viewer->updateShapePose(id,pose); render_window->Render();}
    inline void updateCoordPose(const std::string &id, const Eigen::Affine3f& pose)
    {viewer->updateCoordinateSystemPose(id,pose); render_window->Render();}
    inline void updateCloudPose(const std::string &id, const Eigen::Affine3f& pose)
    {viewer->updatePointCloudPose(id,pose); render_window->Render();}

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //remove
    inline void removeCloud(const std::string &id)
    { viewer->removePointCloud(id); render_window->Render();}
    inline void removeCoord(const std::string &id)
    { viewer->removeCoordinateSystem(id); render_window->Render();}
    inline void removePolygonMesh(const std::string &id)
    { viewer->removePolygonMesh(id); render_window->Render();}
    inline void removeShape(const std::string &id)
    { viewer->removeShape(id); render_window->Render();}
    inline void removeText3D(const std::string &id)
    { viewer->removeText3D(id); render_window->Render();}
    inline void removeCorrespondences(const std::string &id)
    { viewer->removeCorrespondences(id); render_window->Render();}
    inline void removeAllClouds()
    { viewer->removeAllPointClouds(); render_window->Render();}
    inline void removeAllShapes()
    { viewer->removeAllShapes(); render_window->Render();}
    inline void removeAllCoords()
    { viewer->removeAllCoordinateSystems(); render_window->Render();}

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //properties
    void setCloudSelected(const bool selected, const std::string &id);
    void setCloudColor(const Cloud::Ptr &cloud,const std::string &id,double r, double g, double b);
    void setCloudColor(const Cloud::Ptr &cloud,const std::string &id,const std::string &axis);
    void resetCloudColor(const Cloud::Ptr &cloud,const std::string &id);
    void setCloudSize(const std::string &id,float size);
    void setCloudOpacity(const std::string &id,float value);
    void setBackgroundColor(double r, double g, double b);
    void resetBackgroundColor();
    void setShapeColor(const std::string &shapeid,double r, double g, double b);
    void setShapeSize(const std::string &shapeid,float size);
    void setShapeOpacity(const std::string &shapeid,float value);
    void setShapeLineWidth(const std::string &shapeid,float value);
    void setShapeFontSize(const std::string &shapeid,float value);
    void setShapeRepersentation(const std::string &shapeid,int type);
    void setShapeShading(const std::string &shapeid,int type);
    void setSurfaceForAllShapes();
    void setPointsForAllShapes();
    void setWireframeForAllShapes();

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //camera
    inline void updateCamera()
    { viewer->updateCamera(); render_window->Render();}
    inline void resetCamera()
    {viewer->resetCamera();render_window->Render();}
    inline void resetCameraViewpoint (const std::string &id)
    {viewer->resetCameraViewpoint(id); render_window->Render();}
    inline Eigen::Affine3f getViewerPose()
    { return viewer->getViewerPose();}
    inline void initCameraParameters()
    {viewer->initCameraParameters();render_window->Render();}
    inline void loadCameraParameters(const std::string &file)
    { viewer->loadCameraParameters(file);render_window->Render();}
    void setViewerPose(const Eigen::Affine3f &pose);
    inline void setCameraParameters(const Eigen::Matrix3f &intrinsics, const Eigen::Matrix4f &extrinsics)
    {viewer->setCameraParameters(intrinsics,extrinsics);render_window->Render();}
    inline void setCameraClipDistances(double _near, double _far)
    {viewer->setCameraClipDistances(_near,_far); render_window->Render();}
    inline void setCameraFieldOfView (double fovy)
    {viewer->setCameraFieldOfView(fovy);render_window->Render();}
    inline void saveCameraParameters (const std::string &file)
    {viewer->saveCameraParameters(file);}

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //display
    void showInfo(const std::string &text,int height,const std::string &id);
    void showId(const std::string &id);
    void setShowId(const bool &enable);
    inline void setShowFPS(const bool &enable)
    {viewer->setShowFPS(enable);render_window->Render();}
    inline void setShowAxes(const bool &enable)
    {axes->SetEnabled(enable);render_window->Render();}

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //interactor
    void setInteractorEnable(const bool &enable);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //other
    inline bool contains(const std::string &id)
    {return viewer->contains(id);}
    inline void saveScreenshot(const std::string &file)
    { viewer->saveScreenshot(file);}
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //viewport
    inline void setTopview()
    {viewer->setCameraPosition(0,0,0, 0, -1, 0, 0, 0, -1); render_window->Render();}
    inline void setFrontview()
    {viewer->setCameraPosition(0,0,0, 0, 0, -1, 0, 1, 0); render_window->Render();}
    inline void setLeftSideview()
    {viewer->setCameraPosition(0,0,0, 1, 0, 0, 0, 1, 0); render_window->Render();}
    inline void setBackview()
    {viewer->setCameraPosition(0,0,0, 0, 0, 1, 0, 1, 0); render_window->Render();}
    inline void setRightSideview()
    {viewer->setCameraPosition(0,0,0, -1, 0, 0, 0, 1, 0); render_window->Render();}
    inline void setBottomview()
    {viewer->setCameraPosition(0,0,0, 0, 1, 0, 0, 0, 1); render_window->Render();}

protected:
    void dragEnterEvent(QDragEnterEvent *event);
    void dropEvent(QDropEvent *event);
    void resizeEvent(QResizeEvent* size);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

signals:
    void dropFilePath(const QStringList &filepath);
    void sizeChanged(const QSize& size);
    void mouseLeftPressed(const Point2D &pos);
    void mouseLeftReleased(const Point2D &pos);
    void mouseRightPressed(const Point2D &pos);
    void mouseRightReleased(const Point2D &pos);
    void mouseMoved(const Point2D &pos);

private:
    bool showId_enable;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    vtkSmartPointer<vtkRenderer> render;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> render_window;
    vtkSmartPointer<vtkOrientationMarkerWidget> axes;
};

#endif // CLOUDVIEW_H

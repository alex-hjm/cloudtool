/**
 * @file cloudview.h
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#ifndef CT_BASE_CLOUDVIEW_H
#define CT_BASE_CLOUDVIEW_H

#include <QVTKOpenGLNativeWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkOrientationMarkerWidget.h>

#include "base/cloud.h"
#include "base/exports.h"

namespace ct
{

    struct Coord
    {
        QString id = "reference";
        double scale = 1.0;
        Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    };

    struct PointXY
    {
        PointXY(int x, int y) : x(x), y(y) {}
        bool operator==(const PointXY &pt) const
        {
            return (this->x == pt.x) && (this->y == pt.y);
        }
        bool operator!=(const PointXY &pt) const { return !(*this == pt); }
        float x = 0.0f;
        float y = 0.0f;
    };

    class CT_EXPORT CloudView : public QVTKOpenGLNativeWidget
    {
        Q_OBJECT
    public:
        explicit CloudView(QWidget *parent = nullptr);

        /////////////////////////////////////////////////////////////////////////////
        // add
        /**
         * @brief 添加点云
         */
        void addPointCloud(const Cloud::Ptr &cloud);

        /**
         * @brief 添加坐标系
         */
        void addCoordinateSystem(const Coord &coord);

        /**
         * @brief 添加文本
         */
        void addText(const QString &text, int xpos, int ypos,
                     const QString &id = "text", int fontsize = 11, int r = 255,
                     int g = 255, int b = 255);

        /**
         * @brief 添加3D文本
         */
        void addText3D(const QString &text, const PointXYZRGBN &position,
                       const QString &id = "text3d", double textScale = 10,
                       int r = 255, int g = 255, int b = 255);

        /**
         * @brief 添加点云法线
         */
        void addPointCloudNormals(const Cloud::Ptr &cloud, int level, float scale);

        /**
         * @brief 添加多边形网格
         */
        void addPolygonMesh(const pcl::PolygonMesh::Ptr &polymesh,
                            const QString &id = "polygon");

        /**
         * @brief 添加多边形网格线
         */
        void addPolylineFromPolygonMesh(const pcl::PolygonMesh::Ptr &polymesh,
                                        const QString &id = "polyline");

        /**
         * @brief 添加点对的对应关系
         */
        void addCorrespondences(const Cloud::Ptr &source_points,
                                const Cloud::Ptr &target_points,
                                const pcl::CorrespondencesPtr &correspondences,
                                const QString &id = "correspondences");

        /**
         * @brief 添加多边形
         */
        void addPolygon(const Cloud::Ptr &cloud, const QString &id = "polygon",
                        int r = 255, int g = 255, int b = 255);

        /**
         * @brief 添加线段
         */
        void addLine(const PointXYZRGBN &pt1, const PointXYZRGBN &pt2,
                     const QString &id = "line", int r = 255, int g = 255,
                     int b = 255);

        /**
         * @brief 添加箭头
         */
        void addArrow(const PointXYZRGBN &pt1, const PointXYZRGBN &pt2,
                      const QString &id = "arrow", bool display_length = false,
                      int r = 255, int g = 255, int b = 255);

        /**
         * @brief 添加球体
         */
        void addSphere(const PointXYZRGBN &center, double radius,
                       const QString &id = "sphere", int r = 255, int g = 255,
                       int b = 255);

        /**
         * @brief 添加圆柱体
         */
        void addCylinder(const pcl::ModelCoefficients::Ptr &coefficients,
                         const QString &id = "cylinder");

        /**
         * @brief 添加球体
         */
        void addSphere(const pcl::ModelCoefficients::Ptr &coefficients,
                       const QString &id = "sphere");

        /**
         * @brief 添加线段
         */
        void addLine(const pcl::ModelCoefficients::Ptr &coefficients,
                     const QString &id = "line");

        /**
         * @brief 添加平面
         */
        void addPlane(const pcl::ModelCoefficients::Ptr &coefficients,
                      const QString &id = "plane", double x = 0.0, double y = 0.0,
                      double z = 0.0);

        /**
         * @brief 添加圆
         */
        void addCircle(const pcl::ModelCoefficients::Ptr &coefficients,
                       const QString &id = "circle");

        /**
         * @brief 添加圆锥体
         */
        void addCone(const pcl::ModelCoefficients::Ptr &coefficients,
                     const QString &id = "cone");

        /**
         * @brief 添加立方体
         */
        void addCube(const pcl::ModelCoefficients::Ptr &coefficients,
                     const QString &id = "cude");

        /**
         * @brief 添加立方体
         */
        void addCube(const PointXYZRGBN &min, PointXYZRGBN &max,
                     const QString &id = "cude", int r = 255, int g = 255,
                     int b = 255);

        /**
         * @brief 添加立方体
         */
        void addCube(const Box &box, const QString &id = "cude");

        /**
         * @brief 添加点云包围盒
         */
        void addBox(const Cloud::Ptr &cloud);

        /////////////////////////////////////////////////////////////////////////////
        // 2d->3d(display to world)
        /**
         * @brief 屏幕2D坐标映射为
         */
        PointXYZRGBN displayToWorld(const PointXY &xy);

        /**
         * @brief 空间3D坐标映射为屏幕2D坐标
         */
        PointXY worldToDisplay(const PointXYZRGBN &xyz);

        /**
         * @brief 添加相对屏幕的2D线段 -
         */
        void addLine2D(const PointXY &start, const PointXY &end,
                       const QString &id = "line", int r = 255, int g = 255,
                       int b = 255);

        /**
         * @brief 添加相对屏幕的2D多边形
         */
        void addPolygon2D(const std::vector<PointXY> &points,
                          const QString &id = "polyline", int r = 255, int g = 255,
                          int b = 255);

        /**
         * @brief 添加相对屏幕的2D箭头
         */
        void addArrow2D(const PointXY &start, const PointXY &end,
                        const QString &id = "arrow", int r = 255, int g = 255,
                        int b = 255);

        // point pick
        /**
         * @brief 单点选取
         * @param p 屏幕2D坐标点
         * @return int 选中点云的点索引
         */
        int singlePick(const PointXY &p);

        /**
         * @brief 多边形选取
         * @param points 屏幕2D多边形顶点
         * @param cloud  选取的点云
         * @param in_out 选择是否反向
         * @return std::vector<int> 选中点云的点索引集合
         */
        std::vector<int> areaPick(const std::vector<PointXY> &points,
                                  const Cloud::Ptr &cloud, bool in_out = false);

        /////////////////////////////////////////////////////////////////////////////
        // update pose
        /**
         * @brief 更新模型姿态
         */
        void updateShapePose(const QString &id, const Eigen::Affine3f &pose);

        /**
         * @brief 更新坐标系姿态
         */
        void updateCoordinateSystemPose(const QString &id,
                                        const Eigen::Affine3f &pose);

        /**
         * @brief 更新点云姿态
         */
        void updateCloudPose(const QString &id, const Eigen::Affine3f &pose);

        /////////////////////////////////////////////////////////////////////////////
        // remove
        /**
         * @brief 移除点云
         */
        void removePointCloud(const QString &id);

        /**
         * @brief 移除坐标系
         */
        void removeCoordinateSystem(const QString &id);

        /**
         * @brief 移除多边形网格
         */
        void removePolygonMesh(const QString &id);

        /**
         * @brief 移除模型
         */
        void removeShape(const QString &id);

        /**
         * @brief 移除3D文本
         */
        void removeText3D(const QString &id);

        /**
         * @brief 移除对应关系
         */
        void removeCorrespondences(const QString &id);

        /**
         * @brief 移除所有点云
         */
        void removeAllPointClouds();

        /**
         * @brief 移除所有模型
         */
        void removeAllShapes();

        /**
         * @brief 移除所有坐标系
         */
        void removeAllCoordinateSystems();

        /////////////////////////////////////////////////////////////////////////////
        // properties
        /**
         * @brief 设置点云被选中
         */
        void setPointCloudSelected(const bool selected, const QString &id);

        /**
         * @brief 设置点云颜色 (rgb)
         */
        void setPointCloudColor(const Cloud::Ptr &cloud, double r, double g,
                                double b);

        /**
         * @brief 设置点云颜色  (rgb)
         */
        void setPointCloudColor(const QString &id, int r, int g, int b);

        /**
         * @brief 设置点云颜色  (维度)
         */
        void setPointCloudColor(const Cloud::Ptr &cloud, const QString &axis);

        /**
         * @brief 重置置点云颜色
         */
        void resetPointCloudColor(const Cloud::Ptr &cloud);

        /**
         * @brief 设置点云点大小
         */
        void setPointCloudSize(const QString &id, float size);

        /**
         * @brief 设置点云透明度
         */
        void setPointCloudOpacity(const QString &id, float value);

        /**
         * @brief 设置背景颜色
         */
        void setBackgroundColor(double r, double g, double b);

        /**
         * @brief 重置背景颜色
         */
        void resetBackgroundColor();

        /**
         * @brief 设置模型颜色
         */
        void setShapeColor(const QString &shapeid, double r, double g, double b);

        /**
         * @brief 设置模型点大小
         */
        void setShapeSize(const QString &shapeid, float size);

        /**
         * @brief 设置模型透明度
         */
        void setShapeOpacity(const QString &shapeid, float value);

        /**
         * @brief 设置模型线宽
         */
        void setShapeLineWidth(const QString &shapeid, float value);

        /**
         * @brief 设置模型字体大小
         */
        void setShapeFontSize(const QString &shapeid, float value);

        /**
         * @brief 设置模型表示类型
         * @param type 0-点 1-线 2-面
         */
        void setShapeRepersentation(const QString &shapeid, int type);

        /**
         * @brief 设置模型阴影类型
         * @param type 0-FLAT 1-GOURAUD 2-PHONG
         */
        void setShapeShading(const QString &shapeid, int type);

        /**
         * @brief 设置所有模型曲面表示
         */
        void setSurfaceForAllShapes();

        /**
         * @brief 设置所有模型点集表示
         */
        void setPointsForAllShapes();

        /**
         * @brief 设置所有模型线框表示
         */
        void setWireframeForAllShapes();

        /////////////////////////////////////////////////////////////////////////////
        // camera
        /**
         * @brief 更新相机参数
         */
        void updateCamera()
        {
            m_viewer->updateCamera();
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 重置相机参数
         */
        void resetCamera()
        {
            m_viewer->resetCamera();
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 将相机方向从 {0, 0, 0} 重置为给定数据集的 center_{x, y, z}
         */
        void resetCameraViewpoint(const QString &id)
        {
            m_viewer->resetCameraViewpoint(id.toStdString());
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 获取当前视角位姿
         */
        Eigen::Affine3f getm_viewerPose() { return m_viewer->getViewerPose(); }

        /**
         * @brief 初始化相机参数
         */
        void initCameraParameters() { m_viewer->initCameraParameters(); }

        /**
         * @brief 导入相机参数
         */
        void loadCameraParameters(const QString &file)
        {
            m_viewer->loadCameraParameters(file.toStdString());
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 设置视角位姿
         *
         * @param pose
         */
        void setViewerPose(const Eigen::Affine3f &pose);

        /**
         * @brief 设置相机参数
         * @param intrinsics 相机内参
         * @param extrinsics 相机外参
         */
        void setCameraParameters(const Eigen::Matrix3f &intrinsics,
                                 const Eigen::Matrix4f &extrinsics)
        {
            m_viewer->setCameraParameters(intrinsics, extrinsics);
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 设置相机剪切距离
         * @param near 近剪裁距离（不会绘制比这更接近相机的物体）
         * @param far 远剪裁距离（不会绘制比这更远的物体到相机）
         */
        void setCameraClipDistances(double clip_near, double clip_far)
        {
            m_viewer->setCameraClipDistances(clip_near, clip_far);
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 设置相机垂直视野
         * @param fovy 以弧度为单位的垂直视野
         */
        void setCameraFieldOfView(double fovy)
        {
            m_viewer->setCameraFieldOfView(fovy);
            m_viewer->getRenderWindow()->Render();
        }
        /**
         * @brief 将相机参数作为 .cam 文件保存到磁盘。
         */
        void saveCameraParameters(const QString &file)
        {
            m_viewer->saveCameraParameters(file.toStdString());
        }

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        // display
        /**
         * @brief 显示信息
         * @param level 信息位置 1-10
         */
        void showInfo(const QString &text, int level, int r = 255, int g = 255,
                      int b = 255);
        /**
         * @brief 清除信息
         */
        void clearInfo();

        /**
         * @brief 显示点云ID
         */
        void showCloudId(const QString &id);

        /**
         * @brief 设置是否显示点云ID
         */
        void setShowId(const bool &enable);

        /**
         * @brief 设置是否显示帧率
         */
        void setShowFPS(const bool &enable) { m_viewer->setShowFPS(enable); }

        /**
         * @brief 设置是否显示坐标系小部件
         */
        void setShowAxes(const bool &enable)
        {
            m_axes->SetEnabled(enable);
            m_viewer->getRenderWindow()->Render();
        }

        /////////////////////////////////////////////////////////////////////////////

        // other
        /**
         * @brief 检查具有给定ID的点云、模型或坐标是否已添加到视图中。
         */
        bool contains(const QString &id)
        {
            return m_viewer->contains(id.toStdString());
        }

        /**
         * @brief 保存当前视图的PNG格式截图
         */
        void saveScreenshot(const QString &file)
        {
            m_viewer->saveScreenshot(file.toStdString());
        }

        /**
         * @brief 设置是否开启交互
         */
        void setInteractorEnable(const bool &enable);

        ///////////////////////////////////////////////////////////////////////////////////////////////////
        // viewport
        /**
         * @brief 设置为俯视图
         */
        void setTopView()
        {
            m_viewer->setCameraPosition(0, 0, 0, 0, -1, 0, 0, 0, -1);
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 设置为正视图
         */
        void setFrontView()
        {
            m_viewer->setCameraPosition(0, 0, 0, 0, 0, -1, 0, 1, 0);
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 设置为左视图
         */
        void setLeftSideView()
        {
            m_viewer->setCameraPosition(0, 0, 0, 1, 0, 0, 0, 1, 0);
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 设置为后视图
         */
        void setBackView()
        {
            m_viewer->setCameraPosition(0, 0, 0, 0, 0, 1, 0, 1, 0);
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 设置为右视图
         */
        void setRightSideView()
        {
            m_viewer->setCameraPosition(0, 0, 0, -1, 0, 0, 0, 1, 0);
            m_viewer->getRenderWindow()->Render();
        }

        /**
         * @brief 设置为仰视图
         */
        void setBottomView()
        {
            m_viewer->setCameraPosition(0, 0, 0, 0, 1, 0, 0, 0, 1);
            m_viewer->getRenderWindow()->Render();
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        void dragEnterEvent(QDragEnterEvent *event);
        void dropEvent(QDropEvent *event);
        void resizeEvent(QResizeEvent *size);
        void mousePressEvent(QMouseEvent *event);
        void mouseReleaseEvent(QMouseEvent *event);
        void mouseMoveEvent(QMouseEvent *event);

    signals:
        void dropFilePath(const QStringList &filepath);
        void sizeChanged(const QSize &size);
        void mouseLeftPressed(const PointXY &pt);
        void mouseLeftReleased(const PointXY &pt);
        void mouseRightPressed(const PointXY &pt);
        void mouseRightReleased(const PointXY &pt);
        void mouseMoved(const PointXY &pt);

    private:
        Q_DISABLE_COPY(CloudView);
        bool m_show_id;
        int m_info_level;
        QString m_last_id;
        pcl::visualization::PCLVisualizer::Ptr m_viewer;
        vtkSmartPointer<vtkRenderer> m_render;
        vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renderwindow;
        vtkSmartPointer<vtkOrientationMarkerWidget> m_axes;
    };
} // namespace ct

#endif // CT_BASE_CLOUDVIEW_H

/**
 * @file cloudview.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-08
 */
#include "base/cloudview.h"

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2)
VTK_MODULE_INIT(vtkRenderingFreeType)

#include <pcl/geometry/planar_polygon.h>
#include <vtkAxesActor.h>
#include <vtkPointPicker.h>

#include <QDropEvent>
#include <QMimeData>
#include <QUrl>

#define INFO_CLOUD_ID   "info_cloud_id"
#define INFO_TEXT       "info_text"

namespace ct
{
    namespace
    {
        class PCLDisableInteractorStyle : public vtkInteractorStyleTrackballCamera
        {
        public:
            static PCLDisableInteractorStyle* New();
            vtkTypeMacro(PCLDisableInteractorStyle, vtkInteractorStyleTrackballCamera);
            virtual void OnLeftButtonDown() override {}
            virtual void OnMiddleButtonDown() override {}
            virtual void OnRightButtonDown() override {}
            virtual void OnMouseWheelForward() override {}
            virtual void OnMouseWheelBackward() override {}
        };
        vtkStandardNewMacro(PCLDisableInteractorStyle);
    } // namespace

    CloudView::CloudView(QWidget* parent)
        : QVTKOpenGLNativeWidget(parent),
        m_info_level(0),
        m_show_id(true),
        m_last_id(""),
        m_render(vtkSmartPointer<vtkRenderer>::New()),
        m_renderwindow(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New()),
        m_axes(vtkSmartPointer<vtkOrientationMarkerWidget>::New())
    {
        m_renderwindow->AddRenderer(m_render);
        m_viewer.reset(new pcl::visualization::PCLVisualizer(m_render, m_renderwindow, "viewer", false));
        this->SetRenderWindow(m_viewer->getRenderWindow());
        m_viewer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
        m_viewer->setBackgroundColor((double)150.0 / 255.0, (double)150.0 / 255.0, (double)150.0 / 255.0);
        vtkSmartPointer<vtkAxesActor> actor = vtkSmartPointer<vtkAxesActor>::New();
        m_axes->SetOutlineColor(0.9300, 0.5700, 0.1300);
        m_axes->SetOrientationMarker(actor);
        m_axes->SetInteractor(m_viewer->getRenderWindow()->GetInteractor());
        m_axes->SetViewport(0.9, 0, 1, 0.15);
        m_axes->SetEnabled(true);
        m_axes->InteractiveOn();
        m_axes->InteractiveOff();
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPointCloud(const Cloud::Ptr& cloud)
    {
        if (!m_viewer->contains(cloud->id().toStdString()))
            m_viewer->addPointCloud<PointXYZRGBN>(cloud, cloud->id().toStdString());
        else
        {
            pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN> rgb(cloud);
            m_viewer->updatePointCloud<PointXYZRGBN>(cloud, rgb, cloud->id().toStdString());
        }
        if (cloud->pointSize() != 1)
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                       cloud->pointSize(), cloud->id().toStdString());
        if (cloud->opacity() != 1)
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                                       cloud->opacity(), cloud->id().toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPointCloudFromRangeImage(const pcl::RangeImage::Ptr& image, const QString& id, const RGB& rgb)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color(image, rgb.r, rgb.g, rgb.b);
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addPointCloud(image, range_image_color, id.toStdString());
        else
            m_viewer->updatePointCloud(image, range_image_color, id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCoordinateSystem(const Coord& coord)
    {
        m_viewer->removeCoordinateSystem(coord.id.toStdString());
        if (coord.scale != 0)
            m_viewer->addCoordinateSystem(coord.scale, coord.pose, coord.id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addText(const QString& text, int xpos, int ypos, const QString& id, int fontsize, const RGB& rgb)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addText(text.toStdString(), xpos, ypos, fontsize, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        else
            m_viewer->updateText(text.toStdString(), xpos, ypos, fontsize, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addText3D(const QString& text, const PointXYZRGBN& position, const QString& id, double textScale, const RGB& rgb)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addText3D(text.toStdString(), position, textScale, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addText3D(text.toStdString(), position, textScale, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPointCloudNormals(const Cloud::Ptr& cloud, int level, float scale)
    {
        if (!m_viewer->contains(cloud->normalId().toStdString()))
            m_viewer->addPointCloudNormals<PointXYZRGBN>(cloud, level, scale, cloud->normalId().toStdString());
        else
        {
            m_viewer->removePointCloud(cloud->normalId().toStdString());
            m_viewer->addPointCloudNormals<PointXYZRGBN>(cloud, level, scale, cloud->normalId().toStdString());
        }
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                                   cloud->normalColor().rf(), cloud->normalColor().gf(),
                                                   cloud->normalColor().bf(), cloud->normalId().toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPolygonMesh(const pcl::PolygonMesh::Ptr& polymesh, const QString& id)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addPolygonMesh(*polymesh, id.toStdString());
        else
        {
            m_viewer->removePolygonMesh(id.toStdString());
            m_viewer->addPolygonMesh(*polymesh, id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPolylineFromPolygonMesh(const pcl::PolygonMesh::Ptr& polymesh, const QString& id)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addPolylineFromPolygonMesh(*polymesh, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addPolylineFromPolygonMesh(*polymesh, id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCorrespondences(const Cloud::Ptr& source_points, const Cloud::Ptr& target_points,
                                       const pcl::CorrespondencesPtr& correspondences, const QString& id)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addCorrespondences<PointXYZRGBN>(source_points, target_points, *correspondences, id.toStdString());
        else
            m_viewer->updateCorrespondences<PointXYZRGBN>(source_points, target_points, *correspondences, id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPolygon(const Cloud::Ptr& cloud, const QString& id, const RGB& rgb)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addPolygon<PointXYZRGBN>(cloud, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addPolygon<PointXYZRGBN>(cloud, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addLine(const PointXYZRGBN& pt1, const PointXYZRGBN& pt2, const QString& id, const RGB& rgb)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addLine(pt1, pt2, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addLine(pt1, pt2, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addArrow(const PointXYZRGBN& pt1, const PointXYZRGBN& pt2, const QString& id, bool display_length, const RGB& rgb)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addArrow(pt1, pt2, rgb.rf(), rgb.gf(), rgb.bf(), display_length, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addArrow(pt1, pt2, rgb.rf(), rgb.gf(), rgb.bf(), display_length, id.toStdString());
            m_viewer->getRenderWindow()->Render();
        }
    }

    void CloudView::addSphere(const PointXYZRGBN& center, double radius, const QString& id, const RGB& rgb)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addSphere(center, radius, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        else
            m_viewer->updateSphere(center, radius, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCylinder(const pcl::ModelCoefficients::Ptr& coefficients, const QString& id)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addCylinder(*coefficients, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addCylinder(*coefficients, id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addSphere(const pcl::ModelCoefficients::Ptr& coefficients, const QString& id)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addSphere(*coefficients, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addSphere(*coefficients, id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addLine(const pcl::ModelCoefficients::Ptr& coefficients, const QString& id)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addLine(*coefficients, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addLine(*coefficients, id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addPlane(const pcl::ModelCoefficients::Ptr& coefficients,
                             const QString& id, double x, double y, double z)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addPlane(*coefficients, x, y, z, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addPlane(*coefficients, x, y, z, id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCircle(const pcl::ModelCoefficients::Ptr& coefficients, const QString& id)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addCircle(*coefficients, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addCircle(*coefficients, id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCone(const pcl::ModelCoefficients::Ptr& coefficients, const QString& id)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addCone(*coefficients, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addCone(*coefficients, id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCube(const pcl::ModelCoefficients::Ptr& coefficients, const QString& id)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addCube(*coefficients, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addCube(*coefficients, id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCube(const PointXYZRGBN& min, PointXYZRGBN& max, const QString& id, const RGB& rgb)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addCube(min.x, max.x, min.y, max.y, min.z, max.z, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addCube(min.x, max.x, min.y, max.y, min.z, max.z, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addCube(const Box& box, const QString& id)
    {
        if (!m_viewer->contains(id.toStdString()))
            m_viewer->addCube(box.translation, box.rotation, box.width, box.height, box.depth, id.toStdString());
        else
        {
            m_viewer->removeShape(id.toStdString());
            m_viewer->addCube(box.translation, box.rotation, box.width, box.height, box.depth, id.toStdString());
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::addBox(const Cloud::Ptr& cloud)
    {
        if (!m_viewer->contains(cloud->boxId().toStdString()))
            m_viewer->addCube(cloud->box().translation, cloud->box().rotation,
                              cloud->box().width, cloud->box().height,
                              cloud->box().depth, cloud->boxId().toStdString());
        else
        {
            m_viewer->removeShape(cloud->boxId().toStdString());
            m_viewer->addCube(cloud->box().translation, cloud->box().rotation,
                              cloud->box().width, cloud->box().height,
                              cloud->box().depth, cloud->boxId().toStdString());
        }
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                              pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                              cloud->boxId().toStdString());
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, cloud->boxColor().rf(),
                                              cloud->boxColor().gf(), cloud->boxColor().bf(), cloud->boxId().toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    PointXYZRGBN CloudView::displayToWorld(const PointXY& pos)
    {
        double point[4];
        m_render->SetDisplayPoint(pos.x, pos.y, 0.1);
        m_render->DisplayToWorld();
        m_render->GetWorldPoint(point);
        return PointXYZRGBN(point[0], point[1], point[2], 0, 0, 0);
    }

    PointXY CloudView::worldToDisplay(const PointXYZRGBN& point)
    {
        double pos[3];
        m_render->SetWorldPoint(point.x, point.y, point.z, 1);
        m_render->WorldToDisplay();
        m_render->GetDisplayPoint(pos);
        return PointXY(pos[0], pos[1]);
    }

    void CloudView::addLine2D(const PointXY& start, const PointXY& end, const QString& id, const RGB& rgb)
    {
        PointXYZRGBN startPoint = this->displayToWorld(start);
        PointXYZRGBN endPoint = this->displayToWorld(end);
        this->addLine(startPoint, endPoint, id, rgb);
    }

    void CloudView::addPolygon2D(const std::vector<PointXY>& points, const QString& id, const RGB& rgb)
    {
        Cloud::Ptr cloud(new Cloud);
        for (auto& i : points)
        {
            PointXYZRGBN point = this->displayToWorld(i);
            cloud->push_back(point);
        }
        this->addPolygon(cloud, id, rgb);
    }

    void CloudView::addArrow2D(const PointXY& start, const PointXY& end, const QString& id, const RGB& rgb)
    {
        PointXYZRGBN startPoint = this->displayToWorld(start);
        PointXYZRGBN endPoint = this->displayToWorld(end);
        this->addArrow(startPoint, endPoint, id, false, rgb);
    }

    int CloudView::singlePick(const PointXY& pos)
    {
        vtkSmartPointer<vtkPointPicker> m_point_picker = vtkSmartPointer<vtkPointPicker>::New();
        m_renderwindow->GetInteractor()->SetPicker(m_point_picker);
        if (!m_point_picker)
            return -1;
        m_renderwindow->GetInteractor()->StartPickCallback();
        vtkRenderer* ren = this->GetInteractor()->FindPokedRenderer(pos.x, pos.y);
        m_point_picker->Pick(pos.x, pos.y, 0.0, ren);
        return (static_cast<int>(m_point_picker->GetPointId()));
    }

    std::vector<int> CloudView::areaPick(const std::vector<PointXY>& points, const Cloud::Ptr& cloud, bool in_out)
    {
        int size = points.size();
        float constant[99], multiple[99];
        int i, j = size - 1;
        for (i = 0; i < size; i++)
        {
            if (points[j].y == points[i].y)
            {
                constant[i] = points[i].x;
                multiple[i] = 0;
            }
            else
            {
                constant[i] = points[i].x - (points[i].y * points[j].x) / (points[j].y - points[i].y) +
                    (points[i].y * points[i].x) / (points[j].y - points[i].y);
                multiple[i] = (points[j].x - points[i].x) / (points[j].y - points[i].y);
            }
            j = i;
        }

        std::vector<int> indices;
        for (size_t i = 0; i < cloud->size(); i++)
        {
            m_render->SetWorldPoint(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 1);
            m_render->WorldToDisplay();
            double p[3];
            m_render->GetDisplayPoint(p);

            bool oddNodes = in_out, current = points[size - 1].y > p[1], previous;
            for (int m = 0; m < size; m++)
            {
                previous = current;
                current = points[m].y > p[1];
                if (current != previous)
                    oddNodes ^= p[1] * multiple[m] + constant[m] < p[0];
            }
            if (oddNodes) indices.push_back(i);
        }
        return indices;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    // update pose
    void CloudView::updateShapePose(const QString& id, const Eigen::Affine3f& pose)
    {
        m_viewer->updateShapePose(id.toStdString(), pose);
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::updateCoordinateSystemPose(const QString& id, const Eigen::Affine3f& pose)
    {
        m_viewer->updateCoordinateSystemPose(id.toStdString(), pose);
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::updateCloudPose(const QString& id, const Eigen::Affine3f& pose)
    {
        m_viewer->updatePointCloudPose(id.toStdString(), pose);
        m_viewer->getRenderWindow()->Render();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    // remove
    void CloudView::removePointCloud(const QString& id)
    {
        m_viewer->removePointCloud(id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeCoordinateSystem(const QString& id)
    {
        m_viewer->removeCoordinateSystem(id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removePolygonMesh(const QString& id)
    {
        m_viewer->removePolygonMesh(id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeShape(const QString& id)
    {
        m_viewer->removeShape(id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeText3D(const QString& id)
    {
        m_viewer->removeText3D(id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeCorrespondences(const QString& id)
    {
        m_viewer->removeCorrespondences(id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeAllPointClouds()
    {
        m_viewer->removeAllPointClouds();
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeAllShapes()
    {
        m_viewer->removeAllShapes();
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::removeAllCoordinateSystems()
    {
        m_viewer->removeAllCoordinateSystems();
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudSelected(const bool selected, const QString& id)
    {
        m_viewer->setPointCloudSelected(selected, id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudColor(const Cloud::Ptr& cloud, const RGB& rgb)
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBN> color(cloud, rgb.r, rgb.g, rgb.b);
        m_viewer->updatePointCloud(cloud, color, cloud->id().toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudColor(const QString& id, const RGB& rgb)
    {
        m_viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, rgb.rf(), rgb.gf(), rgb.bf(), id.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudColor(const Cloud::Ptr& cloud, const QString& axis)
    {
        pcl::visualization::PointCloudColorHandlerGenericField<PointXYZRGBN>
            fieldcolor(cloud, axis.toStdString());
        m_viewer->updatePointCloud(cloud, fieldcolor, cloud->id().toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::resetPointCloudColor(const Cloud::Ptr& cloud)
    {
        pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN> rgb(cloud);
        m_viewer->updatePointCloud(cloud, rgb, cloud->id().toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudSize(const QString& id, float size)
    {
        m_viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id.toStdString(), 0);
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointCloudOpacity(const QString& id, float value)
    {
        m_viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, value, id.toStdString(), 0);
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setBackgroundColor(const RGB& rgb)
    {
        m_viewer->setBackgroundColor(rgb.rf(), rgb.gf(), rgb.bf());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::resetBackgroundColor()
    {
        m_viewer->setBackgroundColor((double)150.0 / 255.0, (double)150.0 / 255.0, (double)150.0 / 255.0);
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeColor(const QString& shapeid, const RGB& rgb)
    {
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                              rgb.rf(), rgb.gf(), rgb.bf(), shapeid.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeSize(const QString& shapeid, float size)
    {
        m_viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size,
            shapeid.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeOpacity(const QString& shapeid, float value)
    {
        m_viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, value, shapeid.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeLineWidth(const QString& shapeid, float value)
    {
        m_viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, value,
            shapeid.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeFontSize(const QString& shapeid, float value)
    {
        m_viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_FONT_SIZE, value,
            shapeid.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeRepersentation(const QString& shapeid, int type)
    {
        m_viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION, type,
            shapeid.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShapeShading(const QString& shapeid, int type)
    {
        m_viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_SHADING, type, shapeid.toStdString());
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setSurfaceForAllShapes()
    {
        m_viewer->setRepresentationToSurfaceForAllActors();
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setPointsForAllShapes()
    {
        m_viewer->setRepresentationToPointsForAllActors();
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setWireframeForAllShapes()
    {
        m_viewer->setRepresentationToWireframeForAllActors();
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setViewerPose(const Eigen::Affine3f& pose)
    {
        Eigen::Vector3f pos_vector = pose * Eigen::Vector3f(0, 0, 0);
        Eigen::Vector3f look_at_vector =
            pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
        Eigen::Vector3f up_vector = pose.rotation() * Eigen::Vector3f(0, -1, 0);
        m_viewer->setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
                                    look_at_vector[0], look_at_vector[1], look_at_vector[2],
                                    up_vector[0], up_vector[1], up_vector[2]);
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::showInfo(const QString& text, int level, const RGB& rgb)
    {
        m_info_level = std::max(m_info_level, level);
        std::string id = INFO_TEXT + std::to_string(level);
        if (!m_viewer->contains(id))
            m_viewer->addText(text.toStdString(), 10, this->height() - 25 * level, 12, rgb.rf(), rgb.gf(), rgb.bf(), id);
        else
            m_viewer->updateText(text.toStdString(), 10, this->height() - 25 * level, 12, rgb.rf(), rgb.gf(), rgb.bf(), id);
        connect(this, &CloudView::sizeChanged, [=](QSize size)
                { m_viewer->updateText(text.toStdString(), 10, size.height() - 25 * level, 12, rgb.rf(), rgb.gf(), rgb.bf(), id); });
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::clearInfo()
    {
        for (int i = 0; i < m_info_level; i++)
            if (m_viewer->contains(INFO_TEXT + std::to_string(i + 1)))
                m_viewer->removeShape(INFO_TEXT + std::to_string(i + 1));
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::showCloudId(const QString& id)
    {
        m_last_id = id;
        if (!m_show_id)
            return;
        if (!m_viewer->contains(INFO_CLOUD_ID))
            m_viewer->addText(id.toStdString(), this->width() - id.length() * 6 - 20, this->height() - 25, 12, 1, 1, 1, INFO_CLOUD_ID);
        else
            m_viewer->updateText(id.toStdString(), this->width() - id.length() * 6 - 20, this->height() - 25, 12, 1, 1, 1, INFO_CLOUD_ID);
        connect(this, &CloudView::sizeChanged, [=](QSize size)
                { m_viewer->updateText(id.toStdString(), size.width() - id.length() * 6 - 20, size.height() - 25, 12, 1, 1, 1, INFO_CLOUD_ID); });
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setShowId(const bool& enable)
    {
        m_show_id = enable;
        if (enable)
            showCloudId(m_last_id);
        else
            m_viewer->removeShape(INFO_CLOUD_ID);
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::setInteractorEnable(const bool& enable)
    {
        if (!enable)
        {
            vtkNew<PCLDisableInteractorStyle> style;
            m_renderwindow->GetInteractor()->SetInteractorStyle(style);
        }
        else
        {
            vtkNew<pcl::visualization::PCLVisualizerInteractorStyle> style;
            m_renderwindow->GetInteractor()->SetInteractorStyle(style);
        }
        m_viewer->getRenderWindow()->Render();
    }

    void CloudView::dragEnterEvent(QDragEnterEvent* event)
    {
        if (event->mimeData()->hasUrls())
            event->acceptProposedAction();
        else
            event->ignore();
    }

    void CloudView::dropEvent(QDropEvent* event)
    {
        const QMimeData* mimeData = event->mimeData();
        if (mimeData->hasUrls())
        {
            QStringList path;
            QList<QUrl> urlList = mimeData->urls();
            for (auto& url : urlList)
            {
                QString filepath = url.toLocalFile();
                if (!filepath.isEmpty())
                    path.push_back(filepath);
            }
            emit dropFilePath(path);
        }
    }

    void CloudView::resizeEvent(QResizeEvent* size)
    {
        emit sizeChanged(size->size());
        return QVTKOpenGLNativeWidget::resizeEvent(size);
    }

    void CloudView::mousePressEvent(QMouseEvent* event)
    {
        if (event->button() == Qt::LeftButton)
        {
            emit mouseLeftPressed(PointXY(m_renderwindow->GetInteractor()->GetEventPosition()[0],
                                          m_renderwindow->GetInteractor()->GetEventPosition()[1]));
        }
        else if (event->button() == Qt::RightButton)
        {
            emit mouseRightPressed(PointXY(m_renderwindow->GetInteractor()->GetEventPosition()[0],
                                           m_renderwindow->GetInteractor()->GetEventPosition()[1]));
        }
        return QVTKOpenGLNativeWidget::mousePressEvent(event);
    }

    void CloudView::mouseReleaseEvent(QMouseEvent* event)
    {
        if (event->button() == Qt::LeftButton)
        {
            emit viewerPose(m_viewer->getViewerPose());
            emit mouseLeftReleased(PointXY(m_renderwindow->GetInteractor()->GetEventPosition()[0],
                                           m_renderwindow->GetInteractor()->GetEventPosition()[1]));
        }
        else if (event->button() == Qt::RightButton)
        {
            emit mouseRightReleased(PointXY(m_renderwindow->GetInteractor()->GetEventPosition()[0],
                                            m_renderwindow->GetInteractor()->GetEventPosition()[1]));
        }
        return QVTKOpenGLNativeWidget::mouseReleaseEvent(event);
    }

    void CloudView::mouseMoveEvent(QMouseEvent* event)
    {
        emit mouseMoved(PointXY(m_renderwindow->GetInteractor()->GetEventPosition()[0],
                                m_renderwindow->GetInteractor()->GetEventPosition()[1]));
        return QVTKOpenGLNativeWidget::mouseReleaseEvent(event);
    }

} // namespace ct

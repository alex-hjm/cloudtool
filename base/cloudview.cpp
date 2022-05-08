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

namespace ct {

namespace {
class PCLDisableInteractorStyle : public vtkInteractorStyleTrackballCamera {
 public:
  static PCLDisableInteractorStyle *New();
  vtkTypeMacro(PCLDisableInteractorStyle, vtkInteractorStyleTrackballCamera);
  virtual void OnLeftButtonDown() override {}
  virtual void OnMiddleButtonDown() override {}
  virtual void OnRightButtonDown() override {}
  virtual void OnMouseWheelForward() override {}
  virtual void OnMouseWheelBackward() override {}
};
vtkStandardNewMacro(PCLDisableInteractorStyle);
}  // namespace

CloudView::CloudView(QWidget *parent)
    : QVTKOpenGLNativeWidget(parent),
      info_level_(0),
      show_cloud_id_(true),
      render_(vtkSmartPointer<vtkRenderer>::New()),
      render_window_(vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New()),
      axes_(vtkSmartPointer<vtkOrientationMarkerWidget>::New()) {
  render_window_->AddRenderer(render_);
  viewer_.reset(new pcl::visualization::PCLVisualizer(render_, render_window_,
                                                      "viewer", false));
  this->SetRenderWindow(viewer_->getRenderWindow());
  viewer_->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
  viewer_->setBackgroundColor(150.0 / 255.0, 150.0 / 255.0, 150.0 / 255.0);
  vtkSmartPointer<vtkAxesActor> actor = vtkSmartPointer<vtkAxesActor>::New();
  axes_->SetOutlineColor(0.9300, 0.5700, 0.1300);
  axes_->SetOrientationMarker(actor);
  axes_->SetInteractor(viewer_->getRenderWindow()->GetInteractor());
  axes_->SetViewport(0.9, 0, 1, 0.15);
  axes_->SetEnabled(true);
  axes_->InteractiveOn();
  axes_->InteractiveOff();
  viewer_->getRenderWindow()->Render();
}

void CloudView::addPointCloud(const Cloud::Ptr &cloud) {
  if (!viewer_->contains(cloud->id().toStdString()))
    viewer_->addPointCloud<PointXYZRGBN>(cloud, cloud->id().toStdString());
  else {
    pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN> rgb(cloud);
    viewer_->updatePointCloud<PointXYZRGBN>(cloud, rgb,
                                            cloud->id().toStdString());
  }
  if (cloud->pointSize() != 1)
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud->pointSize(),
        cloud->id().toStdString());
  if (cloud->opacity() != 1)
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_OPACITY, cloud->opacity(),
        cloud->id().toStdString(), 0);
  viewer_->getRenderWindow()->Render();
}

void CloudView::addCoordinateSystem(const Coord &coord) {
  viewer_->removeCoordinateSystem(coord.id.toStdString());
  if (coord.scale != 0)
    viewer_->addCoordinateSystem(coord.scale, coord.pose,
                                 coord.id.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::addText(const QString &text, int xpos, int ypos,
                        const QString &id, int fontsize, int r, int g, int b) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addText(text.toStdString(), xpos, ypos, fontsize, r / 255, g / 255,
                     b / 255, id.toStdString());
  else
    viewer_->updateText(text.toStdString(), xpos, ypos, fontsize, r / 255,
                        g / 255, b / 255, id.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::addText3D(const QString &text, const PointXYZRGBN &position,
                          const QString &id, double textScale, int r, int g,
                          int b) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addText3D(text.toStdString(), position, textScale, r / 255,
                       g / 255, b / 255, id.toStdString());
  else {
    viewer_->removeShape(id.toStdString());
    viewer_->addText3D(text.toStdString(), position, textScale, r / 255,
                       g / 255, b / 255, id.toStdString());
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::addPointCloudNormals(const Cloud::Ptr &cloud, int level,
                                     float scale) {
  if (!viewer_->contains(cloud->normalId().toStdString()))
    viewer_->addPointCloudNormals<PointXYZRGBN>(
        cloud, level, scale, cloud->normalId().toStdString());
  else {
    viewer_->removePointCloud(cloud->normalId().toStdString());
    viewer_->addPointCloudNormals<PointXYZRGBN>(
        cloud, level, scale, cloud->normalId().toStdString());
  }
  if (cloud->normalColor().rgb != 1)
    viewer_->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, cloud->normalColor().r / 255,
        cloud->normalColor().g / 255, cloud->normalColor().b / 255,
        cloud->normalId().toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::addPolygonMesh(const pcl::PolygonMesh::Ptr &polymesh,
                               const QString &id) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addPolygonMesh(*polymesh, id.toStdString());
  else {
    viewer_->removePolygonMesh(id.toStdString());
    viewer_->addPolygonMesh(*polymesh, id.toStdString());
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::addPolylineFromPolygonMesh(
    const pcl::PolygonMesh::Ptr &polymesh, const QString &id) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addPolylineFromPolygonMesh(*polymesh, id.toStdString());
  else {
    viewer_->removeShape(id.toStdString());
    viewer_->addPolylineFromPolygonMesh(*polymesh, id.toStdString());
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::addCorrespondences(
    const Cloud::Ptr &source_points, const Cloud::Ptr &target_points,
    const pcl::CorrespondencesPtr &correspondences, const QString &id) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addCorrespondences<PointXYZRGBN>(
        source_points, target_points, *correspondences, id.toStdString());
  else
    viewer_->updateCorrespondences<PointXYZRGBN>(
        source_points, target_points, *correspondences, id.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::addPolygon(const Cloud::Ptr &cloud, const QString &id, int r,
                           int g, int b) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addPolygon<PointXYZRGBN>(cloud, r / 255, g / 255, b / 255,
                                      id.toStdString());
  else {
    viewer_->removeShape(id.toStdString());
    viewer_->addPolygon<PointXYZRGBN>(cloud, r / 255, g / 255, b / 255,
                                      id.toStdString());
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::addLine(const PointXYZRGBN &pt1, const PointXYZRGBN &pt2,
                        const QString &id, int r, int g, int b) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addLine(pt1, pt2, r / 255, g / 255, b / 255, id.toStdString());
  else {
    viewer_->removeShape(id.toStdString());
    viewer_->addLine(pt1, pt2, r / 255, g / 255, b / 255, id.toStdString());
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::addArrow(const PointXYZRGBN &pt1, const PointXYZRGBN &pt2,
                         const QString &id, bool display_length, int r, int g,
                         int b) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addArrow(pt1, pt2, r / 255, g / 255, b / 255, display_length,
                      id.toStdString());
  else {
    viewer_->removeShape(id.toStdString());
    viewer_->addArrow(pt1, pt2, r / 255, g / 255, b / 255, display_length,
                      id.toStdString());
    viewer_->getRenderWindow()->Render();
  }
}

void CloudView::addSphere(const PointXYZRGBN &center, double radius,
                          const QString &id, int r, int g, int b) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addSphere(center, radius, r / 255, g / 255, b / 255,
                       id.toStdString());
  else
    viewer_->updateSphere(center, radius, r / 255, g / 255, b / 255,
                          id.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::addCylinder(const pcl::ModelCoefficients::Ptr &coefficients,
                            const QString &id) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addCylinder(*coefficients, id.toStdString());
  else {
    viewer_->removeShape(id.toStdString());
    viewer_->addCylinder(*coefficients, id.toStdString());
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::addSphere(const pcl::ModelCoefficients::Ptr &coefficients,
                          const QString &id) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addSphere(*coefficients, id.toStdString());
  else {
    viewer_->removeShape(id.toStdString());
    viewer_->addSphere(*coefficients, id.toStdString());
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::addLine(const pcl::ModelCoefficients::Ptr &coefficients,
                        const QString &id) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addLine(*coefficients, id.toStdString());
  else {
    viewer_->removeShape(id.toStdString());
    viewer_->addLine(*coefficients, id.toStdString());
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::addPlane(const pcl::ModelCoefficients::Ptr &coefficients,
                         const QString &id, double x, double y, double z) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addPlane(*coefficients, x, y, z, id.toStdString());
  else {
    viewer_->removeShape(id.toStdString());
    viewer_->addPlane(*coefficients, x, y, z, id.toStdString());
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::addCircle(const pcl::ModelCoefficients::Ptr &coefficients,
                          const QString &id) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addCircle(*coefficients, id.toStdString());
  else {
    viewer_->removeShape(id.toStdString());
    viewer_->addCircle(*coefficients, id.toStdString());
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::addCone(const pcl::ModelCoefficients::Ptr &coefficients,
                        const QString &id) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addCone(*coefficients, id.toStdString());
  else {
    viewer_->removeShape(id.toStdString());
    viewer_->addCone(*coefficients, id.toStdString());
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::addCube(const pcl::ModelCoefficients::Ptr &coefficients,
                        const QString &id) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addCube(*coefficients, id.toStdString());
  else {
    viewer_->removeShape(id.toStdString());
    viewer_->addCube(*coefficients, id.toStdString());
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::addCube(const PointXYZRGBN &min, PointXYZRGBN &max,
                        const QString &id, int r, int g, int b) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addCube(min.x, max.x, min.y, max.y, min.z, max.z, r / 255, g / 255,
                     b / 255, id.toStdString());
  else {
    viewer_->removeShape(id.toStdString());
    viewer_->addCube(min.x, max.x, min.y, max.y, min.z, max.z, r / 255, g / 255,
                     b / 255, id.toStdString());
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::addCube(const Box &box, const QString &id) {
  if (!viewer_->contains(id.toStdString()))
    viewer_->addCube(box.translation, box.rotation, box.width, box.height,
                     box.depth, id.toStdString());
  else {
    viewer_->removeShape(id.toStdString());
    viewer_->addCube(box.translation, box.rotation, box.width, box.height,
                     box.depth, id.toStdString());
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::addBox(const Cloud::Ptr &cloud) {
  if (!viewer_->contains(cloud->boxId().toStdString()))
    viewer_->addCube(cloud->box().translation, cloud->box().rotation,
                     cloud->box().width, cloud->box().height,
                     cloud->box().depth, cloud->boxId().toStdString());
  else {
    viewer_->removeShape(cloud->boxId().toStdString());
    viewer_->addCube(cloud->box().translation, cloud->box().rotation,
                     cloud->box().width, cloud->box().height,
                     cloud->box().depth, cloud->boxId().toStdString());
  }
  viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
      cloud->boxId().toStdString());
  if (cloud->boxColor().r != 255 || cloud->boxColor().g != 255 ||
      cloud->boxColor().b != 255)
    viewer_->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, cloud->boxColor().r / 255,
        cloud->boxColor().g / 255, cloud->boxColor().b / 255,
        cloud->boxId().toStdString());
  viewer_->getRenderWindow()->Render();
}

PointXYZRGBN CloudView::displayToWorld(const PointXY &pos) {
  double point[4];
  render_->SetDisplayPoint(pos.x, pos.y, 0.1);
  render_->DisplayToWorld();
  render_->GetWorldPoint(point);
  return PointXYZRGBN(point[0], point[1], point[2], 0, 0, 0);
}

PointXY CloudView::worldToDisplay(const PointXYZRGBN &point) {
  double pos[3];
  render_->SetWorldPoint(point.x, point.y, point.z, 1);
  render_->WorldToDisplay();
  render_->GetDisplayPoint(pos);
  return PointXY(pos[0], pos[1]);
}

void CloudView::addLine2D(const PointXY &start, const PointXY &end,
                          const QString &id, int r, int g, int b) {
  PointXYZRGBN startPoint = this->displayToWorld(start);
  PointXYZRGBN endPoint = this->displayToWorld(end);
  this->addLine(startPoint, endPoint, id, r, g, b);
}

void CloudView::addPolygon2D(const std::vector<PointXY> &points,
                             const QString &id, int r, int g, int b) {
  Cloud::Ptr cloud(new Cloud);
  for (auto &i : points) {
    PointXYZRGBN point = this->displayToWorld(i);
    cloud->push_back(point);
  }
  this->addPolygon(cloud, id, r, g, b);
}

void CloudView::addArrow2D(const PointXY &start, const PointXY &end,
                           const QString &id, int r, int g, int b) {
  PointXYZRGBN startPoint = this->displayToWorld(start);
  PointXYZRGBN endPoint = this->displayToWorld(end);
  this->addArrow(startPoint, endPoint, id, r, g, b, false);
}

int CloudView::singlePick(const PointXY &pos) {
  vtkSmartPointer<vtkPointPicker> m_point_picker =
      vtkSmartPointer<vtkPointPicker>::New();
  render_window_->GetInteractor()->SetPicker(m_point_picker);
  if (!m_point_picker) return -1;
  render_window_->GetInteractor()->StartPickCallback();
  vtkRenderer *ren = this->GetInteractor()->FindPokedRenderer(pos.x, pos.y);
  m_point_picker->Pick(pos.x, pos.y, 0.0, ren);
  return (static_cast<int>(m_point_picker->GetPointId()));
}

std::vector<int> CloudView::areaPick(const std::vector<PointXY> &points,
                                     const Cloud::Ptr &cloud, bool in_out) {
  int size = points.size();
  float constant[99], multiple[99];
  int i, j = size - 1;
  for (i = 0; i < size; i++) {
    if (points[j].y == points[i].y) {
      constant[i] = points[i].x;
      multiple[i] = 0;
    } else {
      constant[i] = points[i].x -
                    (points[i].y * points[j].x) / (points[j].y - points[i].y) +
                    (points[i].y * points[i].x) / (points[j].y - points[i].y);
      multiple[i] = (points[j].x - points[i].x) / (points[j].y - points[i].y);
    }
    j = i;
  }

  std::vector<int> indices;
  for (size_t i = 0; i < cloud->size(); i++) {
    render_->SetWorldPoint(cloud->points[i].x, cloud->points[i].y,
                           cloud->points[i].z, 1);
    render_->WorldToDisplay();
    double p[3];
    render_->GetDisplayPoint(p);

    bool oddNodes = in_out, current = points[size - 1].y > p[1], previous;
    for (int m = 0; m < size; m++) {
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
void CloudView::updateShapePose(const QString &id,
                                const Eigen::Affine3f &pose) {
  viewer_->updateShapePose(id.toStdString(), pose);
  viewer_->getRenderWindow()->Render();
}

void CloudView::updateCoordinateSystemPose(const QString &id,
                                           const Eigen::Affine3f &pose) {
  viewer_->updateCoordinateSystemPose(id.toStdString(), pose);
  viewer_->getRenderWindow()->Render();
}

void CloudView::updateCloudPose(const QString &id,
                                const Eigen::Affine3f &pose) {
  viewer_->updatePointCloudPose(id.toStdString(), pose);
  viewer_->getRenderWindow()->Render();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// remove
void CloudView::removePointCloud(const QString &id) {
  viewer_->removePointCloud(id.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::removeCoordinateSystem(const QString &id) {
  viewer_->removeCoordinateSystem(id.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::removePolygonMesh(const QString &id) {
  viewer_->removePolygonMesh(id.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::removeShape(const QString &id) {
  viewer_->removeShape(id.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::removeText3D(const QString &id) {
  viewer_->removeText3D(id.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::removeCorrespondences(const QString &id) {
  viewer_->removeCorrespondences(id.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::removeAllPointClouds() {
  viewer_->removeAllPointClouds();
  viewer_->getRenderWindow()->Render();
}

void CloudView::removeAllShapes() {
  viewer_->removeAllShapes();
  viewer_->getRenderWindow()->Render();
}

void CloudView::removeAllCoordinateSystems() {
  viewer_->removeAllCoordinateSystems();
  viewer_->getRenderWindow()->Render();
}

void CloudView::setPointCloudSelected(const bool selected, const QString &id) {
  viewer_->setPointCloudSelected(selected, id.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::setPointCloudColor(const Cloud::Ptr &cloud, double r, double g,
                                   double b) {
  pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBN> color(cloud, r,
                                                                       g, b);
  viewer_->updatePointCloud(cloud, color, cloud->id().toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::setPointCloudColor(const QString &id, int r, int g, int b) {
  viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_COLOR, r / 255, g / 255, b / 255,
      id.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::setPointCloudColor(const Cloud::Ptr &cloud,
                                   const QString &axis) {
  pcl::visualization::PointCloudColorHandlerGenericField<PointXYZRGBN>
      fieldcolor(cloud, axis.toStdString());
  viewer_->updatePointCloud(cloud, fieldcolor, cloud->id().toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::resetPointCloudColor(const Cloud::Ptr &cloud) {
  pcl::visualization::PointCloudColorHandlerRGBField<PointXYZRGBN> rgb(cloud);
  viewer_->updatePointCloud(cloud, rgb, cloud->id().toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::setPointCloudSize(const QString &id, float size) {
  viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id.toStdString(), 0);
  viewer_->getRenderWindow()->Render();
}

void CloudView::setPointCloudOpacity(const QString &id, float value) {
  viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, value, id.toStdString(), 0);
  viewer_->getRenderWindow()->Render();
}

void CloudView::setBackgroundColor(double r, double g, double b) {
  viewer_->setBackgroundColor(r / 255.0, g / 255.0, b / 255.0);
  viewer_->getRenderWindow()->Render();
}

void CloudView::resetBackgroundColor() {
  viewer_->setBackgroundColor(150.0 / 255.0, 150.0 / 255.0, 150.0 / 255.0);
  viewer_->getRenderWindow()->Render();
}

void CloudView::setShapeColor(const QString &shapeid, double r, double g,
                              double b) {
  viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                       r / 255, g / 255, b / 255,
                                       shapeid.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::setShapeSize(const QString &shapeid, float size) {
  viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size,
      shapeid.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::setShapeOpacity(const QString &shapeid, float value) {
  viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_OPACITY, value, shapeid.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::setShapeLineWidth(const QString &shapeid, float value) {
  viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, value,
      shapeid.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::setShapeFontSize(const QString &shapeid, float value) {
  viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_FONT_SIZE, value,
      shapeid.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::setShapeRepersentation(const QString &shapeid, int type) {
  viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_REPRESENTATION, type,
      shapeid.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::setShapeShading(const QString &shapeid, int type) {
  viewer_->setShapeRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_SHADING, type, shapeid.toStdString());
  viewer_->getRenderWindow()->Render();
}

void CloudView::setSurfaceForAllShapes() {
  viewer_->setRepresentationToSurfaceForAllActors();
  viewer_->getRenderWindow()->Render();
}

void CloudView::setPointsForAllShapes() {
  viewer_->setRepresentationToPointsForAllActors();
  viewer_->getRenderWindow()->Render();
}

void CloudView::setWireframeForAllShapes() {
  viewer_->setRepresentationToWireframeForAllActors();
  viewer_->getRenderWindow()->Render();
}

void CloudView::setViewerPose(const Eigen::Affine3f &pose) {
  Eigen::Vector3f pos_vector = pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector =
      pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = pose.rotation() * Eigen::Vector3f(0, -1, 0);
  viewer_->setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
                             look_at_vector[0], look_at_vector[1],
                             look_at_vector[2], up_vector[0], up_vector[1],
                             up_vector[2]);
  viewer_->getRenderWindow()->Render();
}

void CloudView::showInfo(const QString &text, int level, int r, int g, int b) {
  info_level_ = level;
  std::string id = "infotext_" + std::to_string(level);
  if (!viewer_->contains(id))
    viewer_->addText(text.toStdString(), 10, this->height() - 30 * level, 12,
                     r / 255, g / 255, b / 255, id);
  else
    viewer_->updateText(text.toStdString(), 10, this->height() - 30 * level, 12,
                        r / 255, g / 255, b / 255, id);
  connect(this, &CloudView::sizeChanged, [=](QSize size) {
    viewer_->updateText(text.toStdString(), 10, size.height() - 30 * level, 12,
                        r / 255, g / 255, b / 255, id);
  });
  viewer_->getRenderWindow()->Render();
}

void CloudView::clearInfo() {
  for (int i = 0; i < info_level_; i++)
    if (viewer_->contains("infotext_" + std::to_string(i + 1)))
      viewer_->removeShape("infotext_" + std::to_string(i + 1));
  viewer_->getRenderWindow()->Render();
}

void CloudView::showCloudId(const QString &id) {
  last_id_ = id;
  if (!show_cloud_id_) return;
  if (!viewer_->contains("cloudid-text"))
    viewer_->addText(id.toStdString(), this->width() - id.length() * 6 - 20,
                     this->height() - 30, 12, 1, 1, 1, "cloudid-text");
  else
    viewer_->updateText(id.toStdString(), this->width() - id.length() * 6 - 20,
                        this->height() - 30, 12, 1, 1, 1, "cloudid-text");
  connect(this, &CloudView::sizeChanged, [=](QSize size) {
    viewer_->updateText(id.toStdString(), size.width() - id.length() * 6 - 20,
                        size.height() - 30, 12, 1, 1, 1, "cloudid-text");
  });
  viewer_->getRenderWindow()->Render();
}

void CloudView::setShowId(const bool &enable) {
  show_cloud_id_ = enable;
  if (enable)
    showCloudId(last_id_);
  else
    viewer_->removeShape("cloudid-text");
  viewer_->getRenderWindow()->Render();
}

void CloudView::setInteractorEnable(const bool &enable) {
  if (!enable) {
    vtkNew<PCLDisableInteractorStyle> style;
    render_window_->GetInteractor()->SetInteractorStyle(style);
  } else {
    vtkNew<pcl::visualization::PCLVisualizerInteractorStyle> style;
    render_window_->GetInteractor()->SetInteractorStyle(style);
  }
  viewer_->getRenderWindow()->Render();
}

void CloudView::dragEnterEvent(QDragEnterEvent *event) {
  if (event->mimeData()->hasUrls())
    event->acceptProposedAction();
  else
    event->ignore();
}

void CloudView::dropEvent(QDropEvent *event) {
  const QMimeData *mimeData = event->mimeData();
  if (mimeData->hasUrls()) {
    QStringList path;
    QList<QUrl> urlList = mimeData->urls();
    for (auto &url : urlList) {
      QString filepath = url.toLocalFile();
      if (!filepath.isEmpty()) path.push_back(filepath);
    }
    emit dropFilePath(path);
  }
}

void CloudView::resizeEvent(QResizeEvent *size) {
  emit sizeChanged(size->size());
  return QVTKOpenGLNativeWidget::resizeEvent(size);
}

void CloudView::mousePressEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    emit mouseLeftPressed(
        PointXY(render_window_->GetInteractor()->GetEventPosition()[0],
                render_window_->GetInteractor()->GetEventPosition()[1]));
  } else if (event->button() == Qt::RightButton) {
    emit mouseRightPressed(
        PointXY(render_window_->GetInteractor()->GetEventPosition()[0],
                render_window_->GetInteractor()->GetEventPosition()[1]));
  }
  return QVTKOpenGLNativeWidget::mousePressEvent(event);
}

void CloudView::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton) {
    emit mouseLeftReleased(
        PointXY(render_window_->GetInteractor()->GetEventPosition()[0],
                render_window_->GetInteractor()->GetEventPosition()[1]));
  } else if (event->button() == Qt::RightButton) {
    emit mouseRightReleased(
        PointXY(render_window_->GetInteractor()->GetEventPosition()[0],
                render_window_->GetInteractor()->GetEventPosition()[1]));
  }
  return QVTKOpenGLNativeWidget::mouseReleaseEvent(event);
}

void CloudView::mouseMoveEvent(QMouseEvent *event) {
  emit mouseMoved(
      PointXY(render_window_->GetInteractor()->GetEventPosition()[0],
              render_window_->GetInteractor()->GetEventPosition()[1]));
  return QVTKOpenGLNativeWidget::mouseReleaseEvent(event);
}

}  // namespace ct

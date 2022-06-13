#include "pca/surface.h"

#include <pcl/console/time.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/ear_clipping.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/poisson.h>

#include "pca/common.h"

namespace pca {
void Surface::ConcaveHull(double alpha, bool value, int dimensio) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr tree(
      new pcl::search::KdTree<PointXYZRGBN>);
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

  pcl::ConcaveHull<PointXYZRGBN> sur;
  sur.setInputCloud(cloud_);
  sur.setSearchMethod(tree);
  sur.setAlpha(alpha);
  sur.setKeepInformation(value);
  sur.setDimension(dimensio);
  sur.reconstruct(*mesh);
  emit surfaceResult(mesh, time.toc());
}

void Surface::ConvexHull(bool value, int dimensio) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr tree(
      new pcl::search::KdTree<PointXYZRGBN>);
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

  pcl::ConvexHull<PointXYZRGBN> sur;
  sur.setInputCloud(cloud_);
  sur.setSearchMethod(tree);
  sur.setComputeAreaVolume(value);
  sur.setDimension(dimensio);
  sur.reconstruct(*mesh);
  emit surfaceResult(mesh, time.toc());
}

void Surface::EarClipping(pcl::PolygonMesh::Ptr &surface) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr tree(
      new pcl::search::KdTree<PointXYZRGBN>);
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

  pcl::EarClipping sur;
  sur.setInputMesh(surface);
  sur.process(*surface);
  emit surfaceResult(mesh, time.toc());
}

void Surface::GreedyProjectionTriangulation(double mu, int nnn, double radius,
                                            int min, int max, double ep,
                                            bool consistent,
                                            bool consistent_ordering) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr tree(
      new pcl::search::KdTree<PointXYZRGBN>);
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

  pcl::GreedyProjectionTriangulation<PointXYZRGBN> gp3;
  gp3.setInputCloud(cloud_);
  gp3.setSearchMethod(tree);
  gp3.setMu(mu);
  gp3.setMaximumNearestNeighbors(nnn);
  gp3.setMinimumAngle(deg2rad(min));
  gp3.setMaximumAngle(deg2rad(max));
  gp3.setMaximumSurfaceAngle(deg2rad(ep));
  gp3.setSearchRadius(radius);
  gp3.setNormalConsistency(consistent);
  gp3.setConsistentVertexOrdering(consistent_ordering);
  gp3.reconstruct(*mesh);
  emit surfaceResult(mesh, time.toc());
}

void Surface::GridProjection(double resolution, int padding_size, int k,
                             int max_binary_search_level) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr tree(
      new pcl::search::KdTree<PointXYZRGBN>);
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

  pcl::GridProjection<PointXYZRGBN> gp;
  gp.setInputCloud(cloud_);
  gp.setSearchMethod(tree);
  gp.setResolution(resolution);
  gp.setPaddingSize(padding_size);
  gp.setNearestNeighborNum(k);
  gp.setMaxBinarySearchLevel(max_binary_search_level);
  gp.reconstruct(*mesh);
  emit surfaceResult(mesh, time.toc());
}

void Surface::MarchingCubesHoppe(float iso_level, int res_x, int res_y,
                                 int res_z, float percentage,
                                 float dist_ignore) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr tree(
      new pcl::search::KdTree<PointXYZRGBN>);
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

  pcl::MarchingCubesHoppe<PointXYZRGBN> gp;
  gp.setInputCloud(cloud_);
  gp.setSearchMethod(tree);
  gp.setIsoLevel(iso_level);
  gp.setGridResolution(res_x, res_y, res_z);
  gp.setPercentageExtendGrid(percentage);
  gp.setDistanceIgnore(dist_ignore);
  gp.reconstruct(*mesh);
  emit surfaceResult(mesh, time.toc());
}

void Surface::MarchingCubesRBF(float iso_level, int res_x, int res_y, int res_z,
                               float percentage, float epsilon) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr tree(
      new pcl::search::KdTree<PointXYZRGBN>);
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

  pcl::MarchingCubesRBF<PointXYZRGBN> gp;
  gp.setInputCloud(cloud_);
  gp.setSearchMethod(tree);
  gp.setIsoLevel(iso_level);
  gp.setGridResolution(res_x, res_y, res_z);
  gp.setPercentageExtendGrid(percentage);
  gp.setOffSurfaceDisplacement(epsilon);
  gp.reconstruct(*mesh);
  emit surfaceResult(mesh, time.toc());
}

void Surface::Poisson(int depth, int min_depth, float point_weight, float scale,
                      int solver_divide, int iso_divide, float samples_per_node,
                      bool confidence, bool output_polygons, bool manifold) {
  pcl::console::TicToc time;
  time.tic();
  pcl::search::KdTree<PointXYZRGBN>::Ptr tree(
      new pcl::search::KdTree<PointXYZRGBN>);
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);

  pcl::Poisson<PointXYZRGBN> po;
  po.setInputCloud(cloud_);
  po.setSearchMethod(tree);
  po.setDepth(depth);
  po.setMinDepth(min_depth);
  po.setPointWeight(point_weight);
  po.setScale(scale);
  po.setSolverDivide(solver_divide);
  po.setIsoDivide(iso_divide);
  po.setSamplesPerNode(samples_per_node);
  po.setConfidence(confidence);
  po.setOutputPolygons(output_polygons);
  po.setManifold(manifold);
  po.reconstruct(*mesh);
  emit surfaceResult(mesh, time.toc());
}

}  // namespace pca

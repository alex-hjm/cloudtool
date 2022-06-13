#include "surface.h"

Surface::Surface(QObject *parent) : QObject(parent)
{

}

void
Surface::greedyProjectionTriangulation(const Cloud::Ptr &cloud, double mu, int nnn, double radius,int min,int max,int surface)
{

    pcl::console::TicToc time;
    time.tic();
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    pcl::GreedyProjectionTriangulation<PointXYZRGBN> gp3;
    gp3.setMu(mu);
    gp3.setMaximumNearestNeighbors(nnn);
    gp3.setMinimumAngle(min*M_PI/180);
    gp3.setMaximumAngle(max*M_PI/180);
    gp3.setMaximumSurfaceAngle(surface*M_PI/180);
    gp3.setSearchRadius(radius);
    gp3.setInputCloud(cloud);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(*triangles);
    emit result(triangles,cloud->id,time.toc());
}

void
Surface::gridProjection(const Cloud::Ptr &cloud, double resolution,int k, int max_binary_search_level)
{
    pcl::console::TicToc time;
    time.tic();
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    pcl::GridProjection<PointXYZRGBN> gp;
    gp.setResolution(resolution);
    gp.setNearestNeighborNum(k);
    gp.setMaxBinarySearchLevel(max_binary_search_level);
    gp.setInputCloud(cloud);
    gp.setSearchMethod(tree);
    gp.reconstruct (*triangles);
    emit result(triangles,cloud->id,time.toc());
}

void
Surface::poisson(const Cloud::Ptr &cloud, int depth, int min_depth, float point_weight,float scale, float samples_per_node)
{
    pcl::console::TicToc time;
    time.tic();
    pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    pcl::Poisson<PointXYZRGBN> po;
    po.setDepth(depth);
    po.setMinDepth(min_depth);
    po.setPointWeight(point_weight);
    po.setScale(scale);
    po.setSamplesPerNode(samples_per_node);
    po.setInputCloud(cloud);
    po.setSearchMethod(tree);
    po.reconstruct (*triangles);
    emit result(triangles,cloud->id,time.toc());
}

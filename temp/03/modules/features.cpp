#include "features.h"

CloudNormal::Ptr
Features::NormalEstimation(const Cloud::Ptr &cloud,double radius)
{
    pcl::console::TicToc time;
    time.tic();
    pcl::NormalEstimationOMP<PointXYZRGBN, Normal> est_normal;
    CloudNormal::Ptr normals(new CloudNormal);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    est_normal.setInputCloud(cloud);
    est_normal.setSearchMethod(tree);
    est_normal.setRadiusSearch(radius);
    est_normal.setNumberOfThreads(12);
    est_normal.compute(*normals);
    return normals;
}

FPFHFeature::Ptr
Features::FPFHEstimation(const Cloud::Ptr& cloud,double radius)
{
    FPFHFeature::Ptr feature(new FPFHFeature);
    pcl::FPFHEstimationOMP<PointXYZRGBN,Normal,pcl::FPFHSignature33> fpfh;
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
    fpfh.setSearchMethod(tree);
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(cloud->normals);
    fpfh.setRadiusSearch(radius);
    fpfh.setNumberOfThreads(12);
    fpfh.compute(*feature);
    return feature;
}




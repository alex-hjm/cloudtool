#include "tree.h"

Tree::Tree()
{

}

CloudXYZRGBN::Ptr
Tree::KDTree(const CloudXYZRGBN::Ptr &cloud, int n, int k)
{
    time.tic();
    pcl::KdTreeFLANN<PointXYZRGBN> kdtree;
    CloudXYZRGBN::Ptr cloud_tree(new CloudXYZRGBN);
    kdtree.setInputCloud(cloud);
    PointXYZRGBN searchPoint = cloud->points[n];
    std::vector<int> pointSearch;
    std::vector<float> pointSquaredDistance;
    averageDistance=0;
    if (kdtree.nearestKSearch(searchPoint, k, pointSearch, pointSquaredDistance) > 0){
        for (size_t i = 0; i < pointSearch.size(); ++i) {
            cloud_tree->points.push_back(cloud->points[pointSearch[i]]);
            averageDistance+=pointSquaredDistance[i];
        }
        averageDistance/=pointSearch.size();
    }
    tocTime=time.toc();
    return  cloud_tree;
}

CloudXYZRGBN::Ptr
Tree::KDTree(const CloudXYZRGBN::Ptr &cloud, int n, float radius)
{
    time.tic();
    pcl::KdTreeFLANN<PointXYZRGBN> kdtree;
    CloudXYZRGBN::Ptr cloud_tree(new CloudXYZRGBN);
    kdtree.setInputCloud(cloud);
    PointXYZRGBN searchPoint = cloud->points[n];
    std::vector<int> pointSearch;
    std::vector<float> pointSquaredDistance;
    averageDistance=0;
    if (kdtree.radiusSearch(searchPoint, radius, pointSearch, pointSquaredDistance) > 0){
        for (size_t i = 0; i < pointSearch.size(); ++i) {
            cloud_tree->points.push_back(cloud->points[pointSearch[i]]);
            averageDistance+=pointSquaredDistance[i];
        }
         averageDistance/=pointSearch.size();
    }
    tocTime=time.toc();
    return  cloud_tree;
}

CloudXYZRGBN::Ptr
Tree::OCTree(const CloudXYZRGBN::Ptr &cloud, int n, float r)
{
    time.tic();
    CloudXYZRGBN::Ptr cloud_tree(new CloudXYZRGBN);
    pcl::octree::OctreePointCloudSearch<PointXYZRGBN> octree(r);
    octree.setResolution(r);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    PointXYZRGBN searchPoint = cloud->points[n];
    std::vector<int> pointSearch;
    if (octree.voxelSearch(searchPoint, pointSearch)){
        for (size_t i = 0; i < pointSearch.size(); ++i) {
            cloud_tree->points.push_back(cloud->points[pointSearch[i]]);
        }
    }
    tocTime=time.toc();
    return cloud_tree;
}

CloudXYZRGBN::Ptr
Tree::OCTree(const CloudXYZRGBN::Ptr &cloud, int n,float r,int k)
{
    time.tic();
    CloudXYZRGBN::Ptr cloud_tree(new CloudXYZRGBN);
    pcl::octree::OctreePointCloudSearch<PointXYZRGBN> octree(r);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    PointXYZRGBN searchPoint = cloud->points[n];
    std::vector<int> pointSearch;
    std::vector<float> pointSquaredDistance;
    averageDistance=0;
    if (octree.nearestKSearch(searchPoint, k, pointSearch, pointSquaredDistance) > 0) {
        for (size_t i = 0; i < pointSearch.size(); ++i) {
            cloud_tree->points.push_back(cloud->points[pointSearch[i]]);
            averageDistance+=pointSquaredDistance[i];
        }
        averageDistance/=pointSearch.size();
    }
    tocTime=time.toc();
    return cloud_tree;
}

CloudXYZRGBN::Ptr
Tree::OCTree(const CloudXYZRGBN::Ptr&cloud,int n,float r,float radius)
{
    time.tic();
    CloudXYZRGBN::Ptr cloud_tree(new CloudXYZRGBN);
    pcl::octree::OctreePointCloudSearch<PointXYZRGBN> octree(r);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    PointXYZRGBN searchPoint = cloud->points[n];
    std::vector<int> pointSearch;
    std::vector<float> pointSquaredDistance;
    averageDistance=0;
    if (octree.radiusSearch(searchPoint, radius, pointSearch, pointSquaredDistance) > 0){
        for (size_t i = 0; i < pointSearch.size(); ++i) {
            cloud_tree->points.push_back(cloud->points[pointSearch[i]]);
            averageDistance+=pointSquaredDistance[i];
        }
        averageDistance/=pointSearch.size();
    }
    tocTime=time.toc();
    return cloud_tree;
}

CloudXYZRGBN::Ptr
Tree::OCTree(const CloudXYZRGBN::Ptr &cloud,float r,const Eigen::Vector3f &min_pt, const Eigen::Vector3f &max_pt)
{
    time.tic();
    CloudXYZRGBN::Ptr cloud_tree(new CloudXYZRGBN);
    pcl::octree::OctreePointCloudSearch<PointXYZRGBN> octree(r);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
     std::vector<int> pointSearch;
    if (octree.boxSearch(min_pt, max_pt, pointSearch) > 0){
        for (size_t i = 0; i < pointSearch.size(); ++i) {
            cloud_tree->points.push_back(cloud->points[pointSearch[i]]);
        }
    }
    tocTime=time.toc();
    return cloud_tree;
}

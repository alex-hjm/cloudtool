#include "tree.h"

Cloud::Ptr
Tree::KDTree(const Cloud::Ptr &cloud, int n, int k)
{
    Cloud::Ptr cloud_tree(new Cloud);
    pcl::KdTreeFLANN<PointXYZRGBN> kdtree;
    kdtree.setInputCloud(cloud);
    PointXYZRGBN searchPoint = cloud->points[n];
    std::vector<int> pointSearch;
    std::vector<float> pointSquaredDistance;
    if (kdtree.nearestKSearch(searchPoint, k, pointSearch, pointSquaredDistance) > 0)
        for (size_t i = 0; i < pointSearch.size(); ++i)
            cloud_tree->points.push_back(cloud->points[pointSearch[i]]);
    return  cloud_tree;
}

Cloud::Ptr
Tree::KDTree(const Cloud::Ptr &cloud, int n, float radius)
{
    Cloud::Ptr cloud_tree(new Cloud);
    pcl::KdTreeFLANN<PointXYZRGBN> kdtree;
    kdtree.setInputCloud(cloud);
    PointXYZRGBN searchPoint = cloud->points[n];
    std::vector<int> pointSearch;
    std::vector<float> pointSquaredDistance;
    if (kdtree.radiusSearch(searchPoint, radius, pointSearch, pointSquaredDistance) > 0)
        for (size_t i = 0; i < pointSearch.size(); ++i)
            cloud_tree->points.push_back(cloud->points[pointSearch[i]]);
    return  cloud_tree;
}

Cloud::Ptr
Tree::OCTree(const Cloud::Ptr &cloud, int n, float r)
{
    Cloud::Ptr cloud_tree(new Cloud);
    pcl::octree::OctreePointCloudSearch<PointXYZRGBN> octree(r);
    octree.setResolution(r);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    PointXYZRGBN searchPoint = cloud->points[n];
    std::vector<int> pointSearch;
    if (octree.voxelSearch(searchPoint, pointSearch))
        for (size_t i = 0; i < pointSearch.size(); ++i)
            cloud_tree->points.push_back(cloud->points[pointSearch[i]]);
    return cloud_tree;
}

Cloud::Ptr
Tree::OCTree(const Cloud::Ptr &cloud, int n,float r,int k)
{
    Cloud::Ptr cloud_tree(new Cloud);
    pcl::octree::OctreePointCloudSearch<PointXYZRGBN> octree(r);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    PointXYZRGBN searchPoint = cloud->points[n];
    std::vector<int> pointSearch;
    std::vector<float> pointSquaredDistance;
    if (octree.nearestKSearch(searchPoint, k, pointSearch, pointSquaredDistance) > 0)
        for (size_t i = 0; i < pointSearch.size(); ++i)
            cloud_tree->points.push_back(cloud->points[pointSearch[i]]);
    return cloud_tree;
}

Cloud::Ptr
Tree::OCTree(const Cloud::Ptr&cloud,int n,float r,float radius)
{
    Cloud::Ptr cloud_tree(new Cloud);
    pcl::octree::OctreePointCloudSearch<PointXYZRGBN> octree(r);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    PointXYZRGBN searchPoint = cloud->points[n];
    std::vector<int> pointSearch;
    std::vector<float> pointSquaredDistance;
    if (octree.radiusSearch(searchPoint, radius, pointSearch, pointSquaredDistance) > 0)
        for (size_t i = 0; i < pointSearch.size(); ++i)
            cloud_tree->points.push_back(cloud->points[pointSearch[i]]);
    return cloud_tree;
}

Cloud::Ptr
Tree::OCTree(const Cloud::Ptr &cloud,float r,const Eigen::Vector3f &min_pt, const Eigen::Vector3f &max_pt)
{
    Cloud::Ptr cloud_tree(new Cloud);
    pcl::octree::OctreePointCloudSearch<PointXYZRGBN> octree(r);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    std::vector<int> pointSearch;
    if (octree.boxSearch(min_pt, max_pt,pointSearch) > 0)
        for (size_t i = 0; i < pointSearch.size(); ++i)
            cloud_tree->points.push_back(cloud->points[pointSearch[i]]);
    return cloud_tree;
}

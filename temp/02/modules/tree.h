#ifndef TREE_H
#define TREE_H

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>

#include "src/common/customtype.h"


class Tree
{
public:
    Tree();

    float tocTime;
    float averageDistance;
    CloudXYZRGBN::Ptr KDTree(const CloudXYZRGBN::Ptr&,int n,int k);//k searach
    CloudXYZRGBN::Ptr KDTree(const CloudXYZRGBN::Ptr&,int n,float radius);//r search
    CloudXYZRGBN::Ptr OCTree(const CloudXYZRGBN::Ptr&,int n,float r);//voxel search
    CloudXYZRGBN::Ptr OCTree(const CloudXYZRGBN::Ptr&,int n,float r,int k);//k search
    CloudXYZRGBN::Ptr OCTree(const CloudXYZRGBN::Ptr&,int n,float r,float radius);//r search
    CloudXYZRGBN::Ptr OCTree(const CloudXYZRGBN::Ptr&,float r,const Eigen::Vector3f &min_pt, const Eigen::Vector3f &max_pt);//box search

private:
    pcl::console::TicToc time;
};

#endif // TREE_H

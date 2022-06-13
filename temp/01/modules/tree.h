#ifndef TREE_H
#define TREE_H

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include "common/cloud.h"

class Tree
{
public:
    static Cloud::Ptr KDTree(const Cloud::Ptr&,int n,int k);//k searach
    static Cloud::Ptr KDTree(const Cloud::Ptr&,int n,float radius);//r search
    static Cloud::Ptr OCTree(const Cloud::Ptr&,int n,float r);//voxel search
    static Cloud::Ptr OCTree(const Cloud::Ptr&,int n,float r,int k);//k search
    static Cloud::Ptr OCTree(const Cloud::Ptr&,int n,float r,float radius);//r search
    static Cloud::Ptr OCTree(const Cloud::Ptr&,float r,const Eigen::Vector3f &min_pt, const Eigen::Vector3f &max_pt);//box search
};

#endif // TREE_H

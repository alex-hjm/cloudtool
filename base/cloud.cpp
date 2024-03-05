/**
 * @file cloud.cpp
 * @author alex-hjm (hjmalex@163.com)
 * @version 2.0
 * @date 2024-02-21
 */

#include "cloud.h"

#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/common/angles.h>
#include <pcl/kdtree/kdtree_flann.h>

CT_BEGIN_NAMESPACE

void Cloud::setColor(const RGB& rgb)
{
    std::uint32_t rgb_ =  ((std::uint32_t)rgb.r << 16 | (std::uint32_t)rgb.g << 8 | (std::uint32_t)rgb.b);
    for (auto& i : points) i.rgb = *reinterpret_cast<float*>(&rgb_);
}

void Cloud::updateBBox() 
{
    PointXYZRGBN min_pt, max_pt;
    pcl::getMinMax3D(*this, min_pt, max_pt);
    Eigen::Vector3f center = 0.5f * (min_pt.getVector3fMap() + max_pt.getVector3fMap());
    Eigen::Vector3f whd = max_pt.getVector3fMap() - min_pt.getVector3fMap();
    Eigen::Affine3f affine = Eigen::Affine3f::Identity();
    affine = pcl::getTransformation(center[0], center[1], center[2], 0, 0, 0);
    m_bbox = {whd(0), whd(1), whd(2), affine, center, Eigen::Quaternionf(Eigen::Matrix3f::Identity()) };
}

void Cloud::updateResolution() 
{
    pcl::KdTreeFLANN<PointXYZRGBN> kdtree;
    kdtree.setInputCloud(this->makeShared());
    float totalDistance = 0.0;
    for (size_t i = 0; i < this->points.size(); ++i) {
        PointXYZRGBN searchPoint = this->points[i];
        int K = 2;  
        std::vector<int> indices(K);
        std::vector<float> distances(K);
        kdtree.nearestKSearch(searchPoint, K, indices, distances);
        totalDistance += distances[1];
    }

    m_resolution = totalDistance / static_cast<float>(this->points.size());
}

CT_END_NAMESPACE
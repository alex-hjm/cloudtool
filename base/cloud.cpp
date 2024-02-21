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

CT_BEGIN_NAMESPACE

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

CT_END_NAMESPACE
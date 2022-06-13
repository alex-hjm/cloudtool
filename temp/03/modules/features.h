#ifndef FEATURES_H
#define FEATURES_H
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/features/pfh.h>
#include <pcl/features/fpfh_omp.h>

#ifdef _WIN32
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/features/impl/pfh.hpp>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/features/impl/fpfh_omp.hpp>
#endif
#include "common/cloud.h"

class Features
{
public:
    //normals
    static CloudNormal::Ptr NormalEstimation(const Cloud::Ptr& cloud,double radius);
    //feature
    static FPFHFeature::Ptr FPFHEstimation(const Cloud::Ptr&cloud,double radius);
};

#endif // FEATURES_H

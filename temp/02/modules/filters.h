#ifndef FILTERS_H
#define FILTERS_H

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/mls.h>
//#ifdef _WIN32
//#include <pcl/filters/impl/approximate_voxel_grid.hpp>
//#endif

#include "src/common/customtype.h"



class Filters
{
public:
    Filters();
    float tocTime;
    CloudXYZRGBN::Ptr PassThrough(const CloudXYZRGBN::Ptr& cloud,string field_name,float limit_min,float limit_max,bool negative);
    CloudXYZRGBN::Ptr VoxelGrid(const CloudXYZRGBN::Ptr& cloud,float lx, float ly, float lz,bool negative);
    CloudXYZRGBN::Ptr ApproximateVoxelGrid (const CloudXYZRGBN::Ptr& cloud,float lx, float ly, float lz);
    CloudXYZRGBN::Ptr StatisticalOutlierRemoval (const CloudXYZRGBN::Ptr& cloud,int nr_k,double stddev_mult,bool negative);
    CloudXYZRGBN::Ptr RadiusOutlierRemoval (const CloudXYZRGBN::Ptr& cloud,double radius,int min_pts,bool negative);

private:
    pcl::console::TicToc time;
};

#endif // FILTERS_H

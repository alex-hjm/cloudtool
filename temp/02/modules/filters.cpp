#include "filters.h"

Filters::Filters()
{

}

CloudXYZRGBN::Ptr
Filters::PassThrough(const CloudXYZRGBN::Ptr &cloud,string field_name,float limit_min,float limit_max,bool negative)
{
    time.tic();
    CloudXYZRGBN::Ptr cloud_filtered(new CloudXYZRGBN);
    pcl::PassThrough<PointXYZRGBN> pfilter;
    pfilter.setInputCloud (cloud);
    pfilter.setFilterFieldName (field_name);
    pfilter.setFilterLimits (limit_min, limit_max);
    pfilter.setFilterLimitsNegative(negative);
    pfilter.filter (*cloud_filtered);
    tocTime=time.toc();
    return  cloud_filtered;
}

CloudXYZRGBN::Ptr
Filters::VoxelGrid(const CloudXYZRGBN::Ptr &cloud, float lx, float ly, float lz,bool negative)
{
    time.tic();
    CloudXYZRGBN::Ptr cloud_filtered(new CloudXYZRGBN);
    pcl::VoxelGrid<PointXYZRGBN> vfilter;
    vfilter.setInputCloud(cloud);
    vfilter.setLeafSize(lx, ly, lz);
    vfilter.setFilterLimitsNegative(negative);
    vfilter.filter(*cloud_filtered);
    tocTime=time.toc();
    return  cloud_filtered;
}

CloudXYZRGBN::Ptr
Filters::ApproximateVoxelGrid(const CloudXYZRGBN::Ptr &cloud, float lx, float ly, float lz)
{
    time.tic();
    CloudXYZRGBN::Ptr cloud_filtered(new CloudXYZRGBN);
    pcl::ApproximateVoxelGrid<PointXYZRGBN> avfilter;
    avfilter.setInputCloud(cloud);
    avfilter.setLeafSize(lx, ly, lz);
    avfilter.filter(*cloud_filtered);
    tocTime=time.toc();
    return  cloud_filtered;
}


CloudXYZRGBN::Ptr
Filters::StatisticalOutlierRemoval(const CloudXYZRGBN::Ptr &cloud, int nr_k, double stddev_mult,const bool negative)
{
    time.tic();
    CloudXYZRGBN::Ptr cloud_filtered(new CloudXYZRGBN);
    pcl::StatisticalOutlierRemoval<PointXYZRGBN> sfilter;
    sfilter.setInputCloud(cloud);
    sfilter.setMeanK(nr_k);
    sfilter.setStddevMulThresh(stddev_mult);
    sfilter.setNegative(negative);
    sfilter.filter(*cloud_filtered);
    tocTime=time.toc();
    return  cloud_filtered;
}

CloudXYZRGBN::Ptr
Filters::RadiusOutlierRemoval(const CloudXYZRGBN::Ptr &cloud, double radius, int min_pts,bool negative)
{
    time.tic();
    CloudXYZRGBN::Ptr cloud_filtered(new CloudXYZRGBN);
    pcl::RadiusOutlierRemoval<PointXYZRGBN> rfilter;
    rfilter.setInputCloud(cloud);
    rfilter.setRadiusSearch(radius);
    rfilter.setMinNeighborsInRadius(min_pts);
    rfilter.setNegative(negative);
    rfilter.filter(*cloud_filtered);
    tocTime=time.toc();
    return  cloud_filtered;
}

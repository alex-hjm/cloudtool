#include "filter.h"

Filter::Filter(QObject *parent) : QObject(parent)
{

}

void
Filter::passThrough(const Cloud::Ptr& cloud,const std::string &field_name,float limit_min,float limit_max,bool negative)
{
    pcl::console::TicToc time;
    time.tic();
    Cloud::Ptr cloud_filtered(new Cloud);
    cloud_filtered->copyInfo(cloud);
    pcl::PassThrough<PointXYZRGBN> pfilter;
    pfilter.setInputCloud (cloud);
    pfilter.setFilterFieldName (field_name);
    pfilter.setFilterLimits (limit_min, limit_max);
    pfilter.setFilterLimitsNegative(negative);
    pfilter.filter (*cloud_filtered);
    emit result(cloud_filtered,time.toc());
}

void
Filter::voxelGrid(const Cloud::Ptr &cloud, float lx, float ly, float lz,bool negative)
{
    pcl::console::TicToc time;
    time.tic();
    Cloud::Ptr cloud_filtered(new Cloud);
    cloud_filtered->copyInfo(cloud);
    pcl::VoxelGrid<PointXYZRGBN> vfilter;
    vfilter.setInputCloud(cloud);
    vfilter.setLeafSize(lx, ly, lz);
    vfilter.setFilterLimitsNegative(negative);
    vfilter.filter(*cloud_filtered);
    emit result(cloud_filtered,time.toc());
}

void
Filter::approximateVoxelGrid(const Cloud::Ptr &cloud, float lx, float ly, float lz)
{
    pcl::console::TicToc time;
    time.tic();
    Cloud::Ptr cloud_filtered(new Cloud);
    cloud_filtered->copyInfo(cloud);
    pcl::ApproximateVoxelGrid<PointXYZRGBN> avfilter;
    avfilter.setInputCloud(cloud);
    avfilter.setLeafSize(lx, ly, lz);
    avfilter.filter(*cloud_filtered);
    emit result(cloud_filtered,time.toc());
}

void
Filter::statisticalOutlierRemoval(const Cloud::Ptr &cloud, int nr_k, double stddev_mult,const bool negative)
{
    pcl::console::TicToc time;
    time.tic();
    Cloud::Ptr cloud_filtered(new Cloud);
    cloud_filtered->copyInfo(cloud);
    pcl::StatisticalOutlierRemoval<PointXYZRGBN> sfilter;
    sfilter.setInputCloud(cloud);
    sfilter.setMeanK(nr_k);
    sfilter.setStddevMulThresh(stddev_mult);
    sfilter.setNegative(negative);
    sfilter.filter(*cloud_filtered);
    emit result(cloud_filtered,time.toc());
}

void
Filter::radiusOutlierRemoval(const Cloud::Ptr &cloud, double radius, int min_pts,bool negative)
{
    pcl::console::TicToc time;
    time.tic();
    Cloud::Ptr cloud_filtered(new Cloud);
    cloud_filtered->copyInfo(cloud);
    pcl::RadiusOutlierRemoval<PointXYZRGBN> rfilter;
    rfilter.setInputCloud(cloud);
    rfilter.setRadiusSearch(radius);
    rfilter.setMinNeighborsInRadius(min_pts);
    rfilter.setNegative(negative);
    rfilter.filter(*cloud_filtered);
    emit result(cloud_filtered,time.toc());
}

void
Filter::MovingLeastSquares (const Cloud::Ptr& cloud,bool computer_normals,int polynomial_order,float radius)
{
    pcl::console::TicToc time;
    time.tic();
    Cloud::Ptr cloud_filtered(new Cloud);
    pcl::MovingLeastSquaresOMP<PointXYZRGBN,PointXYZRGBN> mfilter;
    mfilter.setInputCloud(cloud);
    mfilter.setSearchRadius (radius);
    mfilter.setComputeNormals (computer_normals);
    mfilter.setPolynomialOrder (polynomial_order);
    mfilter.process (*cloud_filtered);
    emit result(cloud_filtered,time.toc());
}

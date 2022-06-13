#include "filter.h"

Cloud::Ptr
Filter::PassThrough(const Cloud::Ptr& cloud,int index,float limit_min,float limit_max,bool negative)
{
    std::string field_name;
    switch (index) {
    case 0:
        field_name="x";
        break;
    case 1:
        field_name="y";
        break;
    case 2:
        field_name="z";
        break;
    }
    Cloud::Ptr cloud_filtered(new Cloud);
    pcl::PassThrough<PointXYZRGBN> pfilter;
    pfilter.setInputCloud (cloud);
    pfilter.setFilterFieldName (field_name);
    pfilter.setFilterLimits (limit_min, limit_max);
    pfilter.setFilterLimitsNegative(negative);
    pfilter.filter (*cloud_filtered);
    return cloud_filtered;
}

Cloud::Ptr
Filter::VoxelGrid(const Cloud::Ptr &cloud, float lx, float ly, float lz,bool negative)
{
    Cloud::Ptr cloud_filtered(new Cloud);
    pcl::VoxelGrid<PointXYZRGBN> vfilter;
    vfilter.setInputCloud(cloud);
    vfilter.setLeafSize(lx, ly, lz);
    vfilter.setFilterLimitsNegative(negative);
    vfilter.filter(*cloud_filtered);
    return cloud_filtered;
}

Cloud::Ptr
Filter::PlaneRemoval(const Cloud::Ptr &cloud,double threshold, int max_iterations, bool negative)
{
    Cloud::Ptr cloud_filtered(new Cloud);
    ModelCoefficients::Ptr cofes(new ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointXYZRGBN> sacsegn;

    sacsegn.setModelType(pcl::SacModel::SACMODEL_PLANE);
    sacsegn.setMethodType(pcl::SAC_RANSAC);
    sacsegn.setDistanceThreshold(threshold);
    sacsegn.setMaxIterations(max_iterations);
    sacsegn.setOptimizeCoefficients(true);
    sacsegn.setNumberOfThreads(12);
    sacsegn.setInputCloud(cloud);
    sacsegn.segment(*inliers,*cofes);

    pcl::ExtractIndices<PointXYZRGBN> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (!negative);
    extract.filter(*cloud_filtered);
    return cloud_filtered;
}

Cloud::Ptr
Filter::StatisticalOutlierRemoval(const Cloud::Ptr &cloud, int nr_k, double stddev_mult,const bool negative)
{
    Cloud::Ptr cloud_filtered(new Cloud);
    pcl::StatisticalOutlierRemoval<PointXYZRGBN> sfilter;
    sfilter.setInputCloud(cloud);
    sfilter.setMeanK(nr_k);
    sfilter.setStddevMulThresh(stddev_mult);
    sfilter.setNegative(negative);
    sfilter.filter(*cloud_filtered);
    return cloud_filtered;
}

Cloud::Ptr
Filter::RadiusOutlierRemoval(const Cloud::Ptr &cloud, double radius, int min_pts,bool negative)
{
    Cloud::Ptr cloud_filtered(new Cloud);
    pcl::RadiusOutlierRemoval<PointXYZRGBN> rfilter;
    rfilter.setInputCloud(cloud);
    rfilter.setRadiusSearch(radius);
    rfilter.setMinNeighborsInRadius(min_pts);
    rfilter.setNegative(negative);
    rfilter.filter(*cloud_filtered);
    return cloud_filtered;
}

Cloud::Ptr
Filter::MovingLeastSquares (const Cloud::Ptr& cloud,bool computer_normals,int polynomial_order,float radius)
{
    Cloud::Ptr cloud_filtered(new Cloud);
    pcl::MovingLeastSquaresOMP<PointXYZRGBN,PointXYZRGBN> mfilter;
    mfilter.setInputCloud(cloud);
    mfilter.setSearchRadius (radius);
    mfilter.setComputeNormals (computer_normals);
    mfilter.setPolynomialOrder (polynomial_order);
    mfilter.process (*cloud_filtered);
    return cloud_filtered;
}



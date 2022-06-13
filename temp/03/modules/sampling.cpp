#include "sampling.h"

Cloud::Ptr
Sampling::UniformSampling(const Cloud::Ptr &cloud, double radius)
{
    Cloud::Ptr cloud_sampled(new Cloud);
    pcl::UniformSampling<PointXYZRGBN> us;
    us.setInputCloud(cloud);
    us.setRadiusSearch(radius);
    us.filter(*cloud_sampled);
    return cloud_sampled;
}

Cloud::Ptr
Sampling::DownSampling(const Cloud::Ptr &cloud, float leafsize)
{
    Cloud::Ptr cloud_sampled(new Cloud);
    pcl::VoxelGrid<PointXYZRGBN> ds;
    ds.setInputCloud(cloud);
    ds.setLeafSize(leafsize,leafsize,leafsize);
    ds.filter(*cloud_sampled);
    return cloud_sampled;
}

Cloud::Ptr
Sampling::RandomSampling(const Cloud::Ptr &cloud, int sample,int seed)
{
    Cloud::Ptr cloud_sampled(new Cloud);
    pcl::RandomSample<PointXYZRGBN> rs;
    rs.setInputCloud(cloud);
    rs.setSample(sample);
    rs.setSeed(seed);
    rs.filter(*cloud_sampled);
    return cloud_sampled;
}

Cloud::Ptr
Sampling::SamplingSurfaceNormal(const Cloud::Ptr &cloud, int sample, int seed, float ratio)
{
    Cloud::Ptr cloud_sampled(new Cloud);
    pcl::SamplingSurfaceNormal<PointXYZRGBN> ssn;
    ssn.setInputCloud(cloud);
    ssn.setSample(sample);
    ssn.setSeed(seed);
    ssn.setRatio(ratio);
    ssn.filter(*cloud_sampled);
    return cloud_sampled;
}

Cloud::Ptr
Sampling::NormalSpaceSampling(const Cloud::Ptr &cloud, int sample, int seed, int bins)
{
    Cloud::Ptr cloud_sampled(new Cloud);
    pcl::NormalSpaceSampling<PointXYZRGBN,PointXYZRGBN>nss;
    nss.setInputCloud(cloud);
    nss.setNormals(cloud);
    nss.setSample(sample);
    nss.setSeed(seed);
    nss.setBins(bins,bins,bins);
    nss.filter(*cloud_sampled);
    return cloud_sampled;
}

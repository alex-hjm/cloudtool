#include "samplings.h"

Samplings::Samplings()
{

}

CloudXYZRGBN::Ptr Samplings::UniformSampling(const CloudXYZRGBN::Ptr &cloud, double radius)
{
    time.tic();
    CloudXYZRGBN::Ptr cloud_sampled(new CloudXYZRGBN);
    pcl::UniformSampling<PointXYZRGBN> us;
    us.setInputCloud(cloud);
    us.setRadiusSearch(radius);
    us.filter(*cloud_sampled);
    tocTime=time.toc();
    return cloud_sampled;
}


CloudXYZRGBN::Ptr Samplings::DownSampling(const CloudXYZRGBN::Ptr &cloud, float leafsize)
{
    time.tic();
    CloudXYZRGBN::Ptr cloud_sampled(new CloudXYZRGBN);
    pcl::VoxelGrid<PointXYZRGBN> ds;
    ds.setInputCloud(cloud);
    ds.setLeafSize(leafsize,leafsize,leafsize);
    ds.filter(*cloud_sampled);
    tocTime=time.toc();
    return cloud_sampled;

}

CloudXYZRGBN::Ptr Samplings::RandomSampling(const CloudXYZRGBN::Ptr &cloud, int sample,int seed)
{
    time.tic();
    CloudXYZRGBN::Ptr cloud_sampled(new CloudXYZRGBN);
    pcl::RandomSample<PointXYZRGBN> rs;
    rs.setInputCloud(cloud);
    rs.setSample(sample);
    rs.setSeed(seed);
    rs.filter(*cloud_sampled);
    tocTime=time.toc();
    return cloud_sampled;
}

CloudXYZRGBN::Ptr Samplings::SamplingSurfaceNormal(const CloudXYZRGBN::Ptr &cloud, int sample, int seed, float ratio)
{
    time.tic();
    CloudXYZRGBN::Ptr cloud_sampled(new CloudXYZRGBN);
    pcl::SamplingSurfaceNormal<PointXYZRGBN> ssn;
    ssn.setInputCloud(cloud);
    ssn.setSample(sample);
    ssn.setSeed(seed);
    ssn.setRatio(ratio);
    ssn.filter(*cloud_sampled);
    tocTime=time.toc();
    return cloud_sampled;
}

CloudXYZRGBN::Ptr Samplings::NormalSpaceSampling(const CloudXYZRGBN::Ptr &cloud, int sample, int seed, int bins)
{
    time.tic();
    CloudXYZRGBN::Ptr cloud_sampled(new CloudXYZRGBN);
    pcl::NormalSpaceSampling<PointXYZRGBN,PointXYZRGBN>nss;
    nss.setInputCloud(cloud);
    nss.setNormals(cloud);
    nss.setSample(sample);
    nss.setSeed(seed);
    nss.setBins(bins,bins,bins);
    nss.filter(*cloud_sampled);
    tocTime=time.toc();
    return cloud_sampled;
}

#include "sampling.h"

Sampling::Sampling(QObject *parent) : QObject(parent)
{

}

void Sampling::uniformSampling(const Cloud::Ptr &cloud, double radius)
{
    pcl::console::TicToc time;
    time.tic();
    Cloud::Ptr cloud_sampled(new Cloud);
    cloud_sampled->copyInfo(cloud);
    pcl::UniformSampling<PointXYZRGBN> us;
    us.setInputCloud(cloud);
    us.setRadiusSearch(radius);
    us.filter(*cloud_sampled);
    emit result(cloud_sampled,time.toc());
}


void Sampling::downSampling(const Cloud::Ptr &cloud, float leafsize)
{
    pcl::console::TicToc time;
    time.tic();
    Cloud::Ptr cloud_sampled(new Cloud);
    cloud_sampled->copyInfo(cloud);
    pcl::VoxelGrid<PointXYZRGBN> ds;
    ds.setInputCloud(cloud);
    ds.setLeafSize(leafsize,leafsize,leafsize);
    ds.filter(*cloud_sampled);
    emit result(cloud_sampled,time.toc());
}

void Sampling::randomSampling(const Cloud::Ptr &cloud, int sample,int seed)
{
    pcl::console::TicToc time;
    time.tic();
    Cloud::Ptr cloud_sampled(new Cloud);
    cloud_sampled->copyInfo(cloud);
    pcl::RandomSample<PointXYZRGBN> rs;
    rs.setInputCloud(cloud);
    rs.setSample(sample);
    rs.setSeed(seed);
    rs.filter(*cloud_sampled);
    emit result(cloud_sampled,time.toc());
}

void Sampling::samplingSurfaceNormal(const Cloud::Ptr &cloud, int sample, int seed, float ratio)
{
    pcl::console::TicToc time;
    time.tic();
    Cloud::Ptr cloud_sampled(new Cloud);
    cloud_sampled->copyInfo(cloud);
    pcl::SamplingSurfaceNormal<PointXYZRGBN> ssn;
    ssn.setInputCloud(cloud);
    ssn.setSample(sample);
    ssn.setSeed(seed);
    ssn.setRatio(ratio);
    ssn.filter(*cloud_sampled);
    emit result(cloud_sampled,time.toc());
}

void Sampling::normalSpaceSampling(const Cloud::Ptr &cloud, int sample, int seed, int bins)
{
    pcl::console::TicToc time;
    time.tic();
    Cloud::Ptr cloud_sampled(new Cloud);
    cloud_sampled->copyInfo(cloud);
    pcl::NormalSpaceSampling<PointXYZRGBN,PointXYZRGBN>nss;
    nss.setInputCloud(cloud);
    nss.setNormals(cloud);
    nss.setSample(sample);
    nss.setSeed(seed);
    nss.setBins(bins,bins,bins);
    nss.filter(*cloud_sampled);
    emit result(cloud_sampled,time.toc());
}

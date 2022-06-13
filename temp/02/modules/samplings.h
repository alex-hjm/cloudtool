#ifndef SAMPLINGS_H
#define SAMPLINGS_H
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/filters/normal_space.h>

#include "src/common/customtype.h"
class Samplings
{
public:
    Samplings();
    float tocTime;
    CloudXYZRGBN::Ptr UniformSampling(const CloudXYZRGBN::Ptr& cloud,double radius);
    CloudXYZRGBN::Ptr DownSampling(const CloudXYZRGBN::Ptr& cloud,float leafsize);
    CloudXYZRGBN::Ptr RandomSampling(const CloudXYZRGBN::Ptr& cloud,int sample,int seed);
    CloudXYZRGBN::Ptr SamplingSurfaceNormal(const CloudXYZRGBN::Ptr& cloud,int sample,int seed,float ratio);
    CloudXYZRGBN::Ptr NormalSpaceSampling(const CloudXYZRGBN::Ptr& cloud,int sample,int seed,int bins);
private:
   pcl::console::TicToc time;
//   pcl::UniformSampling<PointXYZRGBN> Us;
//   pcl::RandomSample<PointXYZRGBN> Rs;
//   pcl::VoxelGrid<PointXYZRGBN> Ds;
//   pcl::SamplingSurfaceNormal<PointXYZRGBN> Ssn;
//   pcl::NormalSpaceSampling<PointXYZRGBN,Normal> Nss;
};

#endif // SAMPLINGS_H

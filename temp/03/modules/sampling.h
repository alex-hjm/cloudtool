#ifndef SAMPLING_H
#define SAMPLING_H

#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/filters/normal_space.h>
#include "common/cloud.h"
class Sampling
{
public:
    static Cloud::Ptr UniformSampling(const Cloud::Ptr& cloud,double radius);
    static Cloud::Ptr DownSampling(const Cloud::Ptr& cloud,float leafsize);
    static Cloud::Ptr RandomSampling(const Cloud::Ptr& cloud,int sample,int seed);
    static Cloud::Ptr SamplingSurfaceNormal(const Cloud::Ptr& cloud,int sample,int seed,float ratio);
    static Cloud::Ptr NormalSpaceSampling(const Cloud::Ptr& cloud,int sample,int seed,int bins);
};

#endif // SAMPLING_H

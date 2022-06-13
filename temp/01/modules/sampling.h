#ifndef SAMPLING_H
#define SAMPLING_H
#include <QDebug>
#include <pcl/console/time.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/filters/normal_space.h>
#include "common/cloud.h"
class Sampling:public QObject
{
    Q_OBJECT
public:
    explicit Sampling(QObject *parent = nullptr);

signals:
    void result(const Cloud::Ptr &cloud,float time);

public slots:
    void uniformSampling(const Cloud::Ptr& cloud,double radius);
    void downSampling(const Cloud::Ptr& cloud,float leafsize);
    void randomSampling(const Cloud::Ptr& cloud,int sample,int seed);
    void samplingSurfaceNormal(const Cloud::Ptr& cloud,int sample,int seed,float ratio);
    void normalSpaceSampling(const Cloud::Ptr& cloud,int sample,int seed,int bins);
};

#endif // SAMPLING_H

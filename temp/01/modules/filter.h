#ifndef FILTER_H
#define FILTER_H

#include <QObject>
#include <pcl/console/time.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/mls.h>

#include "common/cloud.h"

class Filter:public QObject
{
    Q_OBJECT
public:
    explicit Filter(QObject *parent = nullptr);
signals:
    void result(const Cloud::Ptr &cloud,float time);
public slots:
    void passThrough(const Cloud::Ptr& cloud,const std::string &field_name,float limit_min,float limit_max,bool negative);
    void voxelGrid(const Cloud::Ptr& cloud,float lx, float ly, float lz,bool negative);
    void approximateVoxelGrid (const Cloud::Ptr& cloud,float lx, float ly, float lz);
    void statisticalOutlierRemoval (const Cloud::Ptr& cloud,int nr_k,double stddev_mult,bool negative);
    void radiusOutlierRemoval (const Cloud::Ptr& cloud,double radius,int min_pts,bool negative);
    void MovingLeastSquares (const Cloud::Ptr& cloud,bool computer_normals,int polynomial_order,float radius);
};

#endif // FILTER_H

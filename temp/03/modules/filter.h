#ifndef FILTER_H
#define FILTER_H

#include <pcl/console/time.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#ifdef _WIN32
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/sample_consensus/impl/sac_model_circle.hpp>
#include <pcl/sample_consensus/impl/sac_model_cylinder.hpp>
#include <pcl/sample_consensus/impl/sac_model_cone.hpp>
#include <pcl/sample_consensus/impl/sac_model_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_sphere.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_normal_parallel_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_parallel_line.hpp>
#include <pcl/sample_consensus/impl/sac_model_perpendicular_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_plane.hpp>
#include <pcl/sample_consensus/impl/sac_model_sphere.hpp>
#endif
#include "common/cloudview.h"

class Filter
{
public:
    static Cloud::Ptr PassThrough(const Cloud::Ptr& cloud,int index,float limit_min,float limit_max,bool negative);
    static Cloud::Ptr VoxelGrid(const Cloud::Ptr& cloud,float lx, float ly, float lz,bool negative);
    static Cloud::Ptr PlaneRemoval(const Cloud::Ptr &cloud,double threshold,int max_iterations,bool negative);
    static Cloud::Ptr StatisticalOutlierRemoval (const Cloud::Ptr& cloud,int nr_k,double stddev_mult,bool negative);
    static Cloud::Ptr RadiusOutlierRemoval (const Cloud::Ptr& cloud,double radius,int min_pts,bool negative);
    static Cloud::Ptr MovingLeastSquares (const Cloud::Ptr& cloud,bool computer_normals,int polynomial_order,float radius);
};
#endif // FILTER_H

#ifndef RECOGNITIONS_H
#define RECOGNITIONS_H
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/recognition/cg/hough_3d.h>
#ifdef _WIN32
#include <pcl/recognition/impl/cg/geometric_consistency.hpp>
#include <pcl/recognition/impl/cg/hough_3d.hpp>
#include <pcl/features/impl/board.hpp>
#endif
#include "src/common/customtype.h"

struct Clusters
{
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> translations;
    std::vector<Correspondences> clustered_corrs;
};

class Recognitions
{
public:
    Recognitions();
    Clusters GeometricConsistencyGrouping(const CloudXYZRGBN::Ptr &modelCloud,const CloudXYZRGBN::Ptr &sceneCloud,
                                          const CorrespondencesPtr &corrs,int threshold,double gc_size);
    Clusters Hough3DGrouping(const CloudXYZRGBN::Ptr &modelCloud,const CloudXYZRGBN::Ptr &sceneCloud,const ReferenceFrame::Ptr &model_rf,
                             const ReferenceFrame::Ptr &scene_rf,const CorrespondencesPtr &corrs,double threshold,double bin_size,
                             bool use_interpolation,bool use_distance_weight);

};

#endif // RECOGNITIONS_H

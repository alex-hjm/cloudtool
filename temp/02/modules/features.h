#ifndef FEATURES_H
#define FEATURES_H
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/features/pfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/features/shot_omp.h>

#include <pcl/features/board.h>
#include <pcl/features/flare.h>
#include <pcl/features/shot_lrf_omp.h>
#ifdef _WIN32
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/features/impl/pfh.hpp>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/features/impl/fpfh_omp.hpp>
#include <pcl/features/impl/vfh.hpp>
#include <pcl/features/impl/shot.hpp>
#include <pcl/features/impl/shot_lrf.hpp>
#include <pcl/features/impl/shot_lrf_omp.hpp>
#include <pcl/features/impl/shot_omp.hpp>
#include <pcl/features/impl/board.hpp>
#include <pcl/features/impl/flare.hpp>
#endif
#include "src/common/customtype.h"

class Features
{
public:
    Features();
    float tocTime;
    //normals
    CloudXYZRGBN::Ptr NormalEstimation(const CloudXYZRGBN::Ptr&,int K,double R,const Eigen::Vector3f &viewpoint);

    //boundingbox
    BoundingBox BoundingBoxAABB(const CloudXYZRGBN::Ptr&);
    BoundingBox BoundingBoxOBB(const CloudXYZRGBN::Ptr&);
    BoundingBox BoundingBoxAdjust(const CloudXYZRGBN::Ptr&,const Eigen::Affine3f &t);

    //LocalReferenceFrameEstimation
    ReferenceFrame::Ptr BOARDLocalReferenceFrameEstimation(const CloudXYZRGBN::Ptr&keypoints ,const CloudXYZRGBN::Ptr&cloud,int k,double radius,
                                                           float tangent_radius,float margin_thresh,bool find_holes);
    ReferenceFrame::Ptr FLARELocalReferenceFrameEstimation(const CloudXYZRGBN::Ptr &keypoints, const CloudXYZRGBN::Ptr &cloud, int k, double radius,
                                                           float tangent_radius, float margin_thresh);
    ReferenceFrame::Ptr SHOTLocalReferenceFrameEstimation(const CloudXYZRGBN::Ptr&keypoints ,const CloudXYZRGBN::Ptr&cloud,int k,double radius);


    //feature
    PFHFeature::Ptr PFHEstimation(const CloudXYZRGBN::Ptr&keypoints,const CloudXYZRGBN::Ptr&cloud,int k,double radius);
    FPFHFeature::Ptr FPFHEstimation(const CloudXYZRGBN::Ptr&keypoints,const CloudXYZRGBN::Ptr&cloud,int k,double radius);
    VFHFeature::Ptr VFHEstimation(const CloudXYZRGBN::Ptr&keypoints,const CloudXYZRGBN::Ptr&cloud,int k,double radius);
    SHOTFeature::Ptr SHOTEstimation(const CloudXYZRGBN::Ptr&keypoints,const CloudXYZRGBN::Ptr&cloud,int k,double radius);

private:
    pcl::console::TicToc time;
};

#endif // FEATURES_H

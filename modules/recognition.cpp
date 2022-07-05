/**
 * @file recognition.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#include "modules/recognition.h"

#ifdef __WIN32__
#include <pcl/features/impl/board.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/recognition/impl/cg/geometric_consistency.hpp>
#include <pcl/recognition/impl/cg/hough_3d.hpp>
#endif

namespace ct
{

    void Recognition::GeometricConsistencyGrouping(int threshold, double gc_size)
    {
        TicToc time;
        time.tic();
        Clusters clusters;
        pcl::GeometricConsistencyGrouping<PointXYZRGBN, PointXYZRGBN> gcg;
        gcg.setInputCloud(model_);
        gcg.setSceneCloud(scene_);
        gcg.setModelSceneCorrespondences(corrs_);
        gcg.setGCSize(gc_size);
        gcg.setGCThreshold(threshold);
        gcg.recognize(clusters.translations, clusters.clustered_corrs);
        emit recognitionResult(clusters, time.toc());
    }

    void Recognition::Hough3DGrouping(double threshold, double bin_size, bool use_interpolation,
                                      bool use_distance_weight, float normals_radius, float rf_adius)
    {
        TicToc time;
        time.tic();
        Clusters clusters;
        pcl::Hough3DGrouping<PointXYZRGBN, PointXYZRGBN, pcl::ReferenceFrame, pcl::ReferenceFrame> h3dg;
        h3dg.setInputCloud(model_);
        h3dg.setSceneCloud(scene_);
        h3dg.setInputRf(model_rf_);
        h3dg.setSceneRf(scene_rf_);
        h3dg.setModelSceneCorrespondences(corrs_);
        h3dg.setHoughBinSize(bin_size);
        h3dg.setHoughThreshold(threshold);
        h3dg.setUseInterpolation(use_interpolation);
        h3dg.setUseDistanceWeight(use_distance_weight);
        h3dg.setLocalRfNormalsSearchRadius(normals_radius);
        h3dg.setLocalRfSearchRadius(rf_adius);
        h3dg.recognize(clusters.translations, clusters.clustered_corrs);
        emit recognitionResult(clusters, time.toc());
    }

}  // namespace pca

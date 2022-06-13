#include "recognitions.h"

Recognitions::Recognitions()
{

}

Clusters
Recognitions::GeometricConsistencyGrouping(const CloudXYZRGBN::Ptr &modelCloud, const CloudXYZRGBN::Ptr &sceneCloud,
                                           const CorrespondencesPtr &corrs, int threshold, double gc_size)
{
    Clusters m_clusters;
    pcl::GeometricConsistencyGrouping<PointXYZRGBN,PointXYZRGBN> gcg;
    gcg.setInputCloud(modelCloud);
    gcg.setSceneCloud(sceneCloud);
    gcg.setModelSceneCorrespondences(corrs);
    gcg.setGCSize(gc_size);
    gcg.setGCThreshold(threshold);
    gcg.recognize(m_clusters.translations,m_clusters.clustered_corrs);
    return m_clusters;
}

Clusters
Recognitions::Hough3DGrouping(const CloudXYZRGBN::Ptr &modelCloud, const CloudXYZRGBN::Ptr &sceneCloud, const ReferenceFrame::Ptr &model_rf,
                              const ReferenceFrame::Ptr &scene_rf, const CorrespondencesPtr &corrs, double threshold, double bin_size,
                              bool use_interpolation, bool use_distance_weight)
{
    Clusters m_clusters;
    pcl::Hough3DGrouping<PointXYZRGBN,PointXYZRGBN,pcl::ReferenceFrame,pcl::ReferenceFrame> h3dg;
    h3dg.setInputCloud(modelCloud);
    h3dg.setSceneCloud(sceneCloud);
    h3dg.setInputRf(model_rf);
    h3dg.setSceneRf(scene_rf);
    h3dg.setModelSceneCorrespondences(corrs);
    h3dg.setHoughBinSize(bin_size);
    h3dg.setHoughThreshold(threshold);
    h3dg.setUseInterpolation(use_interpolation);
    h3dg.setUseDistanceWeight(use_distance_weight);
    h3dg.recognize(m_clusters.translations,m_clusters.clustered_corrs);
    return m_clusters;
}

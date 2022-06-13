#ifndef SEGMENTATION_H
#define SEGMENTATION_H
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#ifdef _WIN32
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pcl/segmentation/impl/region_growing_rgb.hpp>
#include <pcl/segmentation/impl/progressive_morphological_filter.hpp>
#include <pcl/features/impl/don.hpp>
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
#include "features.h"
#include "common/cloudview.h"

class Segmentation:public QObject
{
    Q_OBJECT
public:
    explicit Segmentation(QObject *parent = nullptr);
signals:
    void result(const std::vector<Cloud::Ptr> &cloud,float time);
public slots:
    void SACSegmentation(const Cloud::Ptr &cloud, ModelCoefficients::Ptr &cofes,int model_type, int method_type,double threshold,
                                                   int max_iterations,double min_radius,double max_radius,bool optimize,bool negative);
    void SACSegmentationFromNormals(const Cloud::Ptr &cloud, ModelCoefficients::Ptr &cofes,int model_type, int method_type,double threshold,
                                                              int max_iterations, double min_radius,double max_radius,bool optimize,bool negative,
                                                              double distance_weightconst);
    void EuclideanClusterExtraction(const Cloud::Ptr &cloud,double tolerance,int min_cluster_size,
                                                              int max_cluster_size);
    void RegionGrowing(const Cloud::Ptr &cloud,int min_cluster_size,int max_cluster_size,
                                                 bool smoothMode,bool CurvatureTest,bool ResidualTest,float SmoothnessThreshold,
                                                 float ResidualThreshold,float CurvatureThreshold,int NumberOfNeighbours);
    void RegionGrowingRGB(const Cloud::Ptr &cloud, int min_cluster_size, int max_cluster_size, bool smoothMode,
                                                    bool CurvatureTest, bool ResidualTest, float SmoothnessThreshold, float ResidualThreshold,
                                                    float CurvatureThreshold, int NumberOfNeighbours,float pointcolorthresh,float regioncolorthresh,
                                                    float distancethresh,int nghbr_number);
    void MinCutSegmentation(const Cloud::Ptr &cloud,const Eigen::Vector3f &center,double sigma,double radius,
                                                      double weight,int neighbour_number);
    void DonSegmentation(const Cloud::Ptr &cloud,double mean_radius,double scale1,double scale2,double threshold,
                                                   double segradius,int minClusterSize ,int maxClusterSize);
    void MorphologicalFilter(const Cloud::Ptr &cloud,int max_window_size,float slope,float max_distance,float initial_distance,
                                                       float cell_size,float base,bool negative);
    void SupervoxelClustering(const Cloud::Ptr &cloud,float voxel_resolution,float seed_resolution,float color_importance,
                                                        float spatial_importance,float normal_importance, bool camera_transform);
};

#endif // SEGMENTATION_H

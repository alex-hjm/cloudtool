#ifndef SEGMENTATIONS_H
#define SEGMENTATIONS_H
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
#include "src/common/customtype.h"

struct LinePoint
{
    PointXYZRGBN beginpoint;
    PointXYZRGBN endpoint;
};

class Segmentations
{
public:
    Segmentations();

    float tocTime;

    std::vector<CloudXYZRGBN::Ptr> SACSegmentation(const CloudXYZRGBN::Ptr &cloud, ModelCoefs &cofes,int model_type, int method_type,double threshold,
                                                   int max_iterations,double min_radius,double max_radius,bool optimize,bool negative);
    std::vector<CloudXYZRGBN::Ptr> SACSegmentationFromNormals(const CloudXYZRGBN::Ptr &cloud, ModelCoefs &cofes,int model_type, int method_type,double threshold,
                                                              int max_iterations, double min_radius,double max_radius,bool optimize,bool negative,
                                                              double distance_weightconst);
    std::vector<CloudXYZRGBN::Ptr> EuclideanClusterExtraction(const CloudXYZRGBN::Ptr &cloud,double tolerance,int min_cluster_size,
                                                              int max_cluster_size);
    std::vector<CloudXYZRGBN::Ptr> ConditionalEuclideanClustering(const CloudXYZRGBN::Ptr &cloud,double tolerance,int min_cluster_size,
                                                                  int max_cluster_size,int condition);
    std::vector<CloudXYZRGBN::Ptr> RegionGrowing(const CloudXYZRGBN::Ptr &cloud,int min_cluster_size,int max_cluster_size,
                                                 bool smoothMode,bool CurvatureTest,bool ResidualTest,float SmoothnessThreshold,
                                                 float ResidualThreshold,float CurvatureThreshold,int NumberOfNeighbours);
    std::vector<CloudXYZRGBN::Ptr> RegionGrowingRGB(const CloudXYZRGBN::Ptr &cloud, int min_cluster_size, int max_cluster_size, bool smoothMode,
                                                    bool CurvatureTest, bool ResidualTest, float SmoothnessThreshold, float ResidualThreshold,
                                                    float CurvatureThreshold, int NumberOfNeighbours,float pointcolorthresh,float regioncolorthresh,
                                                    float distancethresh,int nghbr_number);
    std::vector<CloudXYZRGBN::Ptr> MinCutSegmentation(const CloudXYZRGBN::Ptr &cloud,Eigen::Vector3f center,double sigma,double radius,
                                                      double weight,int neighbour_number);
    std::vector<CloudXYZRGBN::Ptr> DonSegmentation(const CloudXYZRGBN::Ptr &cloud,double mean_radius,double scale1,double scale2,double threshold,
                                                   double segradius,int minClusterSize ,int maxClusterSize);
    std::vector<CloudXYZRGBN::Ptr> MorphologicalFilter(const CloudXYZRGBN::Ptr &cloud,int max_window_size,float slope,float max_distance,float initial_distance,
                                                       float cell_size,float base,bool negative);
    std::vector<CloudXYZRGBN::Ptr> SupervoxelClustering(const CloudXYZRGBN::Ptr &cloud,float voxel_resolution,float seed_resolution,float color_importance,
                                                        float spatial_importance,float normal_importance, bool camera_transform);
    std::vector<CloudXYZRGBN::Ptr> getVoxelCentroidPointNormal();
    std::vector<LinePoint> getVoxelConnectLine();


private:
    pcl::console::TicToc time;
    std::vector<CloudXYZRGBN::Ptr> VoxelCentroidPointNormal;
    std::vector<LinePoint> VoxelConnectLine;

    //pcl::ExtractIndices<PointXYZRGBN> extract;
    //pcl::search::KdTree<PointXYZRGBN>::Ptr tree;

    //pcl::SACSegmentation<PointXYZRGBN> sacseg;
    //pcl::SACSegmentationFromNormals<PointXYZRGBN,Normal> sacsegn;
    //pcl::EuclideanClusterExtraction<PointXYZRGBN> ec;
    //pcl::RegionGrowing<PointXYZRGBN, Normal> reg;
   // pcl::RegionGrowingRGB<PointXYZRGBN, Normal> regrgb;
    //pcl::MinCutSegmentation<PointXYZRGBN> mseg;
    //pcl::NormalEstimationOMP<PointXYZRGBN, PointN> ne;
    //pcl::DifferenceOfNormalsEstimation<PointXYZRGBN, PointN, PointN> don;
    //pcl::ConditionOr<PointN>::Ptr range_cond;
    //pcl::ConditionalRemoval<PointN> condrem;
    //pcl::EuclideanClusterExtraction<PointN> ecc;
    //pcl::ProgressiveMorphologicalFilter<PointXYZRGBN> pmf;
};

#endif // SEGMENTATIONS_H

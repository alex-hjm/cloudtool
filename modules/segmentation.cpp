/**
 * @file segmentation.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#include "modules/segmentation.h"

#include "base/common.h"
#include "modules/filters.h"

#include <pcl/console/time.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/don.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/min_cut_segmentation.h>

#include <pcl/segmentation/impl/progressive_morphological_filter.hpp>
#include <pcl/features/impl/don.hpp>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/features/impl/normal_3d_omp.hpp>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/segmentation/impl/extract_polygonal_prism_data.hpp>
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pcl/segmentation/impl/region_growing_rgb.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/segmentation/impl/seeded_hue_segmentation.hpp>
#include <pcl/segmentation/impl/segment_differences.hpp>

namespace ct
{

    std::vector<Cloud::Ptr> Segmentation::getClusters(const IndicesClustersPtr& clusters)
    {
        std::vector<Cloud::Ptr> segmented_clouds;
        PointIndicesPtr segmented_indices(new PointIndices);
        for (IndicesClusters::const_iterator it = clusters->begin();it != clusters->end(); ++it)
        {
            Cloud::Ptr cloud_cluster(new Cloud);
            pcl::copyPointCloud(*cloud_, *it, *cloud_cluster);
            cloud_cluster->width = cloud_cluster->points.size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            segmented_clouds.push_back(cloud_cluster);
            segmented_indices->indices.insert(segmented_indices->indices.end(), it->indices.begin(), it->indices.end());
        }
        if (negative_)
            return getClusters(segmented_indices);
        else
            return segmented_clouds;
    }

    std::vector<Cloud::Ptr> Segmentation::getClusters(const PointIndicesPtr& clusters)
    {
        std::vector<Cloud::Ptr> segmented_clouds;
        Cloud::Ptr cloud_segmented(new Cloud);
        pcl::ExtractIndices<PointXYZRGBN> extract;
        extract.setInputCloud(cloud_);
        extract.setIndices(clusters);
        extract.setNegative(negative_);
        extract.filter(*cloud_segmented);
        segmented_clouds.push_back(cloud_segmented);
        return segmented_clouds;
    }

    void Segmentation::SACSegmentation(int model, int method, double threshold, int max_iterations, double probability,
                                       bool optimize, double min_radius, double max_radius)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::PointIndicesPtr indices(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr cofes(new pcl::ModelCoefficients);

        pcl::SACSegmentation<PointXYZRGBN> sacseg;
        sacseg.setInputCloud(cloud_);
        sacseg.setModelType(model);
        sacseg.setMethodType(method);
        sacseg.setDistanceThreshold(threshold);
        sacseg.setMaxIterations(max_iterations);
        sacseg.setOptimizeCoefficients(optimize);
        sacseg.setRadiusLimits(min_radius, max_radius);
        sacseg.setProbability(probability);
        sacseg.setNumberOfThreads(12);

        sacseg.segment(*indices, *cofes);
        emit segmentationResult(cloud_->id(), getClusters(indices), time.toc(), cofes);
    }

    void Segmentation::SACSegmentationFromNormals(int model, int method, double threshold, int max_iterations,
                                                  double probability, bool optimize, double min_radius, double max_radius,
                                                  double distance_weight, double d)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(
            new pcl::search::KdTree<PointXYZRGBN>);
        pcl::PointIndicesPtr indices(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr cofes(new pcl::ModelCoefficients);

        pcl::SACSegmentationFromNormals<PointXYZRGBN, PointXYZRGBN> sacseg;
        sacseg.setInputCloud(cloud_);
        sacseg.setInputNormals(cloud_);
        sacseg.setModelType(model);
        sacseg.setMethodType(method);
        sacseg.setDistanceThreshold(threshold);
        sacseg.setMaxIterations(max_iterations);
        sacseg.setOptimizeCoefficients(optimize);
        sacseg.setRadiusLimits(min_radius, max_radius);
        sacseg.setProbability(probability);
        sacseg.setNormalDistanceWeight(distance_weight);
        sacseg.setDistanceFromOrigin(d);
        sacseg.setNumberOfThreads(12);

        sacseg.segment(*indices, *cofes);
        emit segmentationResult(cloud_->id(), getClusters(indices), time.toc(), cofes);
    }

    void Segmentation::EuclideanClusterExtraction(double tolerance, int min_cluster_size, int max_cluster_size)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        IndicesClustersPtr clusters(new IndicesClusters);

        pcl::EuclideanClusterExtraction<PointXYZRGBN> seg;
        seg.setInputCloud(cloud_);
        seg.setSearchMethod(tree);
        seg.setClusterTolerance(tolerance);
        seg.setMinClusterSize(min_cluster_size);
        seg.setMaxClusterSize(max_cluster_size);
        seg.extract(*clusters);
        emit segmentationResult(cloud_->id(), getClusters(clusters), time.toc());
    }

    void Segmentation::RegionGrowing(int min_cluster_size, int max_cluster_size, bool smooth_mode, bool curvature_test,
                                     bool residual_test, float smoothness_threshold, float residual_threshold,
                                     float curvature_threshold, int neighbours)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        IndicesClustersPtr clusters(new IndicesClusters);

        pcl::RegionGrowing<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputCloud(cloud_);
        reg.setInputNormals(cloud_);
        reg.setSearchMethod(tree);
        reg.setMinClusterSize(min_cluster_size);
        reg.setMaxClusterSize(max_cluster_size);
        reg.setMinClusterSize(min_cluster_size);
        reg.setMaxClusterSize(max_cluster_size);
        reg.setSearchMethod(tree);
        reg.setSmoothModeFlag(smooth_mode);
        reg.setSmoothnessThreshold(pcl::deg2rad(smoothness_threshold));
        reg.setCurvatureTestFlag(curvature_test);
        reg.setCurvatureThreshold(curvature_threshold);
        reg.setResidualTestFlag(residual_test);
        reg.setResidualThreshold(residual_threshold);
        reg.setNumberOfNeighbours(neighbours);
        reg.extract(*clusters);
        emit segmentationResult(cloud_->id(), getClusters(clusters), time.toc());
    }

    void Segmentation::RegionGrowingRGB(int min_cluster_size, int max_cluster_size, bool smooth_mode,
                                        bool curvature_test, bool residual_test, float smoothness_threshold,
                                        float residual_threshold, float curvature_threshold, int neighbours,
                                        float pt_thresh, float re_thresh, float dis_thresh, int nghbr_number)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        IndicesClustersPtr clusters(new IndicesClusters);

        pcl::RegionGrowingRGB<PointXYZRGBN, PointXYZRGBN> reg;
        reg.setInputCloud(cloud_);
        reg.setInputNormals(cloud_);
        reg.setSearchMethod(tree);
        reg.setMinClusterSize(min_cluster_size);
        reg.setMaxClusterSize(max_cluster_size);
        reg.setMinClusterSize(min_cluster_size);
        reg.setMaxClusterSize(max_cluster_size);
        reg.setSearchMethod(tree);
        reg.setSmoothModeFlag(smooth_mode);
        reg.setSmoothnessThreshold(pcl::deg2rad(smoothness_threshold));
        reg.setCurvatureTestFlag(curvature_test);
        reg.setCurvatureThreshold(curvature_threshold);
        reg.setResidualTestFlag(residual_test);
        reg.setResidualThreshold(residual_threshold);
        reg.setNumberOfNeighbours(neighbours);
        reg.setPointColorThreshold(pt_thresh);
        reg.setRegionColorThreshold(re_thresh);
        reg.setDistanceThreshold(dis_thresh);
        reg.setNumberOfRegionNeighbours(nghbr_number);
        reg.extract(*clusters);
        emit segmentationResult(cloud_->id(), getClusters(clusters), time.toc());
    }

    void Segmentation::SupervoxelClustering(float voxel_resolution, float seed_resolution, float color_importance,
                                            float spatial_importance, float normal_importance, bool camera_transform)
    {
        TicToc time;
        time.tic();

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_xyzrgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::copyPointCloud(*cloud_, *cloud_xyzrgba);

        pcl::SupervoxelClustering<pcl::PointXYZRGBA> super(voxel_resolution, seed_resolution);
        super.setInputCloud(cloud_xyzrgba);
        super.setVoxelResolution(voxel_resolution);
        super.setSeedResolution(seed_resolution);
        super.setColorImportance(color_importance);
        super.setSpatialImportance(spatial_importance);
        super.setNormalImportance(normal_importance);
        super.setUseSingleCameraTransform(camera_transform);

        std::map<std::uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr> supervoxel_clusters;
        super.extract(supervoxel_clusters);

        std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
        super.getSupervoxelAdjacency(supervoxel_adjacency);

        std::vector<Cloud::Ptr> segmented_clouds;

        std::multimap<uint32_t, uint32_t>::iterator label_itr = supervoxel_adjacency.begin();
        for (; label_itr != supervoxel_adjacency.end();)
        {
            uint32_t supervoxel_label = label_itr->first;
            pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);
            Cloud::Ptr supervoxel_cloud(new Cloud);
            pcl::copyPointCloud(*supervoxel->voxels_, *supervoxel_cloud);
            segmented_clouds.push_back(supervoxel_cloud);
            label_itr = supervoxel_adjacency.upper_bound(supervoxel_label);
        }

        emit segmentationResult(cloud_->id(), segmented_clouds, time.toc());
    }
    
    void Segmentation::ConditionalEuclideanClustering(ConditionFunction func, float cluster_tolerance,
                                                      int min_cluster_size, int max_cluster_size)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        IndicesClustersPtr clusters(new IndicesClusters);

        pcl::ConditionalEuclideanClustering<PointXYZRGBN> seg;
        seg.setInputCloud(cloud_);
        seg.setSearchMethod(tree);
        seg.setConditionFunction(func);
        seg.setClusterTolerance(cluster_tolerance);
        seg.setMinClusterSize(min_cluster_size);
        seg.setMaxClusterSize(max_cluster_size);
        seg.segment(*clusters);
        emit segmentationResult(cloud_->id(), getClusters(clusters), time.toc());
    }

    void Segmentation::DonSegmentation(double mean_radius, double scale1, double scale2, double threshold,
                                       double segradius, int minClusterSize, int maxClusterSize)
    {
        TicToc time;
        time.tic();
        IndicesClustersPtr cluster_indices(new IndicesClusters);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        scale1 *= mean_radius;
        scale2 *= mean_radius;
        segradius *= mean_radius;

        pcl::NormalEstimationOMP<PointXYZRGBN, PointN> ne;
        ne.setInputCloud(cloud_);
        ne.setSearchMethod(tree);
        ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());

        pcl::PointCloud<PointN>::Ptr normals_small_scale(new pcl::PointCloud<PointN>);
        pcl::PointCloud<PointN>::Ptr normals_large_scale(new pcl::PointCloud<PointN>);

        ne.setNumberOfThreads(12);
        ne.setRadiusSearch(scale1);
        ne.compute(*normals_small_scale);
        ne.setRadiusSearch(scale2);
        ne.compute(*normals_large_scale);

        pcl::PointCloud<PointN>::Ptr doncloud(new pcl::PointCloud<PointN>);
        pcl::DifferenceOfNormalsEstimation<PointXYZRGBN, PointN, PointN> don;
        don.setInputCloud(cloud_);
        don.setNormalScaleLarge(normals_large_scale);
        don.setNormalScaleSmall(normals_small_scale);
        don.computeFeature(*doncloud);

        pcl::ConditionOr<PointN>::Ptr range_cond;
        range_cond->addComparison(pcl::FieldComparison<PointN>::ConstPtr(
            new pcl::FieldComparison<PointN>("curvature", pcl::ComparisonOps::GT, threshold)));

        pcl::ConditionalRemoval<PointN> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(doncloud);
        pcl::PointCloud<PointN>::Ptr doncloud_filtered(new pcl::PointCloud<PointN>);
        condrem.filter(*doncloud_filtered);

        pcl::search::KdTree<PointN>::Ptr segtree(new pcl::search::KdTree<PointN>);
        segtree->setInputCloud(doncloud_filtered);

        pcl::EuclideanClusterExtraction<PointN> ecc;
        ecc.setClusterTolerance(segradius);
        ecc.setMinClusterSize(minClusterSize);
        ecc.setMaxClusterSize(maxClusterSize);
        ecc.setSearchMethod(segtree);
        ecc.setInputCloud(doncloud_filtered);
        ecc.extract(*cluster_indices);
        emit segmentationResult(cloud_->id(), getClusters(cluster_indices), time.toc());
    }

    void Segmentation::ExtractPolygonalPrismData(const Cloud::Ptr& hull, double height_min, double height_max, float vpx,
                                                 float vpy, float vpz)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        pcl::PointIndicesPtr indices(new pcl::PointIndices);

        pcl::ExtractPolygonalPrismData<PointXYZRGBN> seg;
        seg.setInputCloud(cloud_);
        seg.setInputPlanarHull(hull);
        seg.setHeightLimits(height_min, height_max);
        seg.setViewPoint(vpx, vpy, vpz);
        seg.segment(*indices);
        emit segmentationResult(cloud_->id(), getClusters(indices), time.toc());
    }

    void Segmentation::MinCutSegmentation(double sigma, double radius, double weight, int neighbour_number)
    {
        TicToc time;
        time.tic();
        IndicesClustersPtr cluster_indices(new IndicesClusters);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        Cloud::Ptr foreground_points(new Cloud);
        foreground_points->push_back({ cloud_->center()[0],cloud_->center()[1],cloud_->center()[2] });

        pcl::MinCutSegmentation<PointXYZRGBN> mseg;
        mseg.setInputCloud(cloud_);
        mseg.setSigma(sigma);
        mseg.setRadius(radius);
        mseg.setSourceWeight(weight);
        mseg.setSearchMethod(tree);
        mseg.setForegroundPoints(foreground_points);
        mseg.setNumberOfNeighbours(neighbour_number);
        mseg.extract(*cluster_indices);
        emit segmentationResult(cloud_->id(), getClusters(cluster_indices), time.toc());
    }

    void Segmentation::MorphologicalFilter(int max_window_size, float slope, float max_distance, float initial_distance,
                                           float cell_size, float base)
    {
        TicToc time;
        time.tic();
        PointIndicesPtr inliers(new PointIndices);

        Cloud::Ptr cloud_segmented(new Cloud());
        pcl::ProgressiveMorphologicalFilter<PointXYZRGBN> pmf;

        pmf.setInputCloud(cloud_);
        pmf.setMaxWindowSize(max_window_size);
        pmf.setSlope(slope);
        pmf.setMaxDistance(max_distance);
        pmf.setInitialDistance(initial_distance);
        pmf.setCellSize(cell_size);
        pmf.setBase(base);
        pmf.extract(inliers->indices);

        emit segmentationResult(cloud_->id(), getClusters(inliers), time.toc());
    }

    void Segmentation::SeededHueSegmentation(double tolerance, float delta_hue)
    {
        TicToc time;
        time.tic();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*cloud_, *cloudrgb);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        pcl::PointIndicesPtr indices_in(new pcl::PointIndices);
        pcl::PointIndicesPtr indices_out(new pcl::PointIndices);

        pcl::SeededHueSegmentation seg;
        seg.setInputCloud(cloudrgb);
        seg.setSearchMethod(tree);
        seg.setClusterTolerance(tolerance);
        seg.setDeltaHue(delta_hue);
        seg.segment(*indices_in, *indices_out);
        emit segmentationResult(cloud_->id(), getClusters(indices_out), time.toc());
    }

    void Segmentation::SegmentDifferences(const Cloud::Ptr& tar_cloud, double sqr_threshold)
    {
        TicToc time;
        time.tic();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        Cloud::Ptr diff = cloud_->makeShared();
        pcl::PointIndicesPtr indices(new pcl::PointIndices);

        pcl::SegmentDifferences<PointXYZRGBN> seg;
        seg.setInputCloud(cloud_);
        seg.setSearchMethod(tree);
        seg.setTargetCloud(tar_cloud);
        seg.setDistanceThreshold(sqr_threshold);
        seg.segment(*diff);
        std::vector<Cloud::Ptr> segmented_clouds;
        segmented_clouds.push_back(diff);
        emit segmentationResult(cloud_->id(), segmented_clouds, time.toc());
    }

}  // namespace pca


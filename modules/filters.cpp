/**
 * @file filters.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#include "modules/filters.h"

#include <pcl/console/time.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/local_maximum.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/filters/shadowpoints.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>

namespace ct
{
    void Filters::ApproximateVoxelGrid(float lx, float ly, float lz,
                                       bool downsample)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::ApproximateVoxelGrid<PointXYZRGBN> avfilter;
        avfilter.setInputCloud(cloud_);
        avfilter.setLeafSize(lx, ly, lz);
        avfilter.setDownsampleAllData(downsample);
        avfilter.filter(*cloud_filtered);

        if (negative_)
        {
            pcl::ExtractIndices<PointXYZRGBN> extract;
            extract.setInputCloud(cloud_);
            extract.setIndices(avfilter.getRemovedIndices());
            extract.filter(*cloud_filtered);
        }

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::ConditionalRemoval(Condition::Ptr con)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);

        pcl::ConditionalRemoval<PointXYZRGBN> bfilter;
        bfilter.setInputCloud(cloud_);
        bfilter.setKeepOrganized(keep_organized_);
        bfilter.setUserFilterValue(value_);
        bfilter.setCondition(con);
        bfilter.filter(*cloud_filtered);

        if (negative_)
        {
            pcl::ExtractIndices<PointXYZRGBN> extract;
            extract.setInputCloud(cloud_);
            extract.setIndices(bfilter.getRemovedIndices());
            extract.filter(*cloud_filtered);
        }

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::Convolution3D(float sigma, float sigma_coefficient,
                                float threshold, double radius)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::filters::GaussianKernel<PointXYZRGBN, PointXYZRGBN> kernel;
        kernel.setSigma(sigma);
        kernel.setThresholdRelativeToSigma(sigma_coefficient);
        kernel.setThreshold(threshold);

        pcl::filters::Convolution3D<
            PointXYZRGBN, PointXYZRGBN,
            pcl::filters::GaussianKernel<PointXYZRGBN, PointXYZRGBN>>
            convolution;
        convolution.setInputCloud(cloud_);
        convolution.setKernel(kernel);
        convolution.setNumberOfThreads(14);
        convolution.setSearchMethod(tree);
        convolution.setRadiusSearch(radius);
        convolution.convolve(*cloud_filtered);
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::CropBox(const Eigen::Vector4f& min_pt,
                          const Eigen::Vector4f& max_pt,
                          const Eigen::Affine3f& transform)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::CropBox<PointXYZRGBN> cb;
        cb.setInputCloud(cloud_);
        cb.setMin(min_pt);
        cb.setMax(max_pt);
        cb.setTransform(transform);
        cb.setKeepOrganized(keep_organized_);
        cb.setUserFilterValue(value_);
        cb.setNegative(negative_);
        cb.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::CropHull(const std::vector<pcl::Vertices>& polygons, int dim,
                           bool crop_outside)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::CropHull<PointXYZRGBN> ch;
        ch.setInputCloud(cloud_);
        ch.setHullIndices(polygons);
        ch.setDim(dim);
        ch.setCropOutside(crop_outside);
        ch.setKeepOrganized(keep_organized_);
        ch.setUserFilterValue(value_);
        ch.setNegative(negative_);
        ch.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::FrustumCulling(const Eigen::Matrix4f& camera_pose, float hfov,
                                 float vfov, float np_dist, float fp_dist)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::FrustumCulling<PointXYZRGBN> filter;
        filter.setInputCloud(cloud_);
        filter.setCameraPose(camera_pose);
        filter.setHorizontalFOV(hfov);
        filter.setVerticalFOV(vfov);
        filter.setNearPlaneDistance(np_dist);
        filter.setFarPlaneDistance(fp_dist);
        filter.setKeepOrganized(keep_organized_);
        filter.setUserFilterValue(value_);
        filter.setNegative(negative_);
        filter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::GridMinimum(const float resolution)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::GridMinimum<PointXYZRGBN> filter(resolution);
        filter.setInputCloud(cloud_);
        filter.setResolution(resolution);
        filter.setKeepOrganized(keep_organized_);
        filter.setUserFilterValue(value_);
        filter.setNegative(negative_);
        filter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::LocalMaximum(float radius)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();
        pcl::PointCloud<PointXYZRGB>::Ptr cloud(new pcl::PointCloud<PointXYZRGB>);
        pcl::copyPointCloud(*cloud_, *cloud);

        pcl::LocalMaximum<PointXYZRGB> filter;
        filter.setInputCloud(cloud);
        filter.setRadius(radius);
        filter.setKeepOrganized(keep_organized_);
        filter.setUserFilterValue(value_);
        filter.setNegative(negative_);
        filter.filter(*cloud);

        pcl::copyPointCloud(*cloud, *cloud_filtered);
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::MedianFilter(int window_size, float max_allowed_movement)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::MedianFilter<PointXYZRGBN> filter;
        filter.setInputCloud(cloud_);
        filter.setWindowSize(window_size);
        filter.setMaxAllowedMovement(max_allowed_movement);
        filter.filter(*cloud_filtered);

        if (negative_)
        {
            pcl::ExtractIndices<PointXYZRGBN> extract;
            extract.setInputCloud(cloud_);
            extract.setIndices(filter.getRemovedIndices());
            extract.filter(*cloud_filtered);
        }
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::MovingLeastSquares(bool computer_normals, int polynomial_order,
                                     float radius, double sqr_gauss_param,
                                     int upsampling_method, double uradius,
                                     double step_size, int dradius,
                                     float voxel_size, int iterations,
                                     int projection_method)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);
        typedef pcl::MovingLeastSquares<PointXYZRGBN, PointXYZRGBN>::UpsamplingMethod
            UpsamplingMethod;
        typedef pcl::MLSResult::ProjectionMethod ProjectionMethod;
        pcl::MovingLeastSquaresOMP<PointXYZRGBN, PointXYZRGBN> mfilter;
        mfilter.setInputCloud(cloud_);
        mfilter.setSearchRadius(radius);
        mfilter.setComputeNormals(computer_normals);
        mfilter.setPolynomialOrder(polynomial_order);
        mfilter.setSearchMethod(tree);
        mfilter.setSqrGaussParam(sqr_gauss_param);
        mfilter.setUpsamplingMethod(UpsamplingMethod(upsampling_method));
        mfilter.setUpsamplingRadius(uradius);
        mfilter.setUpsamplingStepSize(step_size);
        mfilter.setPointDensity(dradius);
        mfilter.setDilationVoxelSize(voxel_size);
        mfilter.setDilationIterations(iterations);
        mfilter.setProjectionMethod(ProjectionMethod(projection_method));
        mfilter.process(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::NormalSpaceSampling(int sample, int seed, int binsx, int binsy,
                                      int binsz)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::NormalSpaceSampling<PointXYZRGBN, PointXYZRGBN> filter;
        filter.setInputCloud(cloud_);
        filter.setNormals(cloud_);
        filter.setSample(sample);
        filter.setSeed(seed);
        filter.setBins(binsx, binsy, binsz);
        filter.setKeepOrganized(keep_organized_);
        filter.setUserFilterValue(value_);
        filter.setNegative(negative_);
        filter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::PassThrough(const std::string& field_name, float limit_min,
                              float limit_max)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::PassThrough<PointXYZRGBN> pfilter;
        pfilter.setInputCloud(cloud_);
        pfilter.setFilterFieldName(field_name);
        pfilter.setFilterLimits(limit_min, limit_max);
        pfilter.setFilterLimitsNegative(negative_);
        pfilter.setKeepOrganized(keep_organized_);
        pfilter.setUserFilterValue(value_);
        pfilter.filter(*cloud_filtered);
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::PlaneClipper3D(const Eigen::Vector4f& plane_params)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::PlaneClipper3D<PointXYZRGBN> filter(plane_params);
        filter.setPlaneParameters(plane_params);
        pcl::IndicesPtr indices(new pcl::Indices);
        filter.clipPointCloud3D(*cloud_, *indices);

        pcl::ExtractIndices<PointXYZRGBN> extract;
        extract.setInputCloud(cloud_);
        extract.setIndices(indices);
        extract.setNegative(negative_);
        extract.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::ProjectInliers(int type, const pcl::ModelCoefficients::Ptr& model,
                                 bool va)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();
        pcl::PointCloud<PointXYZRGB>::Ptr cloud_copy(
            new pcl::PointCloud<PointXYZRGB>);
        pcl::PointCloud<PointXYZRGB>::Ptr cloud_copy_filtered(
            new pcl::PointCloud<PointXYZRGB>);
        pcl::copyPointCloud(*cloud_, *cloud_copy);

        pcl::ProjectInliers<PointXYZRGB> filter;
        filter.setInputCloud(cloud_copy);
        filter.setModelType(type);
        filter.setModelCoefficients(model);
        filter.setCopyAllData(va);
        filter.filter(*cloud_copy_filtered);

        if (negative_)
        {
            pcl::ExtractIndices<PointXYZRGB> extract;
            extract.setInputCloud(cloud_copy);
            extract.setIndices(filter.getRemovedIndices());
            extract.filter(*cloud_copy_filtered);
        }
        pcl::copyPointCloud(*cloud_copy_filtered, *cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::RadiusOutlierRemoval(double radius, int min_pts)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::RadiusOutlierRemoval<PointXYZRGBN> rfilter;
        rfilter.setInputCloud(cloud_);
        rfilter.setRadiusSearch(radius);
        rfilter.setMinNeighborsInRadius(min_pts);
        rfilter.setNegative(negative_);
        rfilter.setKeepOrganized(keep_organized_);
        rfilter.setUserFilterValue(value_);
        rfilter.filter(*cloud_filtered);
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::RandomSample(int sample, int seed)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::RandomSample<PointXYZRGBN> rfilter;
        rfilter.setInputCloud(cloud_);
        rfilter.setSample(sample);
        rfilter.setSeed(seed);
        rfilter.setNegative(negative_);
        rfilter.setKeepOrganized(keep_organized_);
        rfilter.setUserFilterValue(value_);
        rfilter.filter(*cloud_filtered);
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::SamplingSurfaceNormal(int sample, int seed, float ratio)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::SamplingSurfaceNormal<PointXYZRGBN> rfilter;
        rfilter.setInputCloud(cloud_);
        rfilter.setSample(sample);
        rfilter.setSeed(seed);
        rfilter.setRatio(ratio);
        rfilter.filter(*cloud_filtered);

        if (negative_)
        {
            pcl::ExtractIndices<PointXYZRGBN> extract;
            extract.setInputCloud(cloud_);
            extract.setIndices(rfilter.getRemovedIndices());
            extract.filter(*cloud_filtered);
        }
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::ShadowPoints(float threshold)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::ShadowPoints<PointXYZRGBN, PointXYZRGBN> rfilter;
        rfilter.setInputCloud(cloud_);
        rfilter.setNormals(cloud_);
        rfilter.setThreshold(threshold);
        rfilter.setNegative(negative_);
        rfilter.setKeepOrganized(keep_organized_);
        rfilter.setUserFilterValue(value_);
        rfilter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::StatisticalOutlierRemoval(int nr_k, double stddev_mult)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::StatisticalOutlierRemoval<PointXYZRGBN> sfilter;
        sfilter.setInputCloud(cloud_);
        sfilter.setMeanK(nr_k);
        sfilter.setStddevMulThresh(stddev_mult);
        sfilter.setNegative(negative_);
        sfilter.setKeepOrganized(keep_organized_);
        sfilter.setUserFilterValue(value_);
        sfilter.filter(*cloud_filtered);
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::UniformSampling(double radius)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::UniformSampling<PointXYZRGBN> sfilter;
        sfilter.setInputCloud(cloud_);
        sfilter.setRadiusSearch(radius);
        sfilter.filter(*cloud_filtered);

        if (negative_)
        {
            pcl::ExtractIndices<PointXYZRGBN> extract;
            extract.setInputCloud(cloud_);
            extract.setIndices(sfilter.getRemovedIndices());
            extract.filter(*cloud_filtered);
        }
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::VoxelGrid(float lx, float ly, float lz, bool downsample,
                            int min_points_per_voxel)
    {
        pcl::console::TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered = cloud_->makeShared();

        pcl::VoxelGrid<PointXYZRGBN> vfilter;
        vfilter.setInputCloud(cloud_);
        vfilter.setLeafSize(lx, ly, lz);
        vfilter.setFilterLimitsNegative(negative_);
        vfilter.setDownsampleAllData(downsample);
        vfilter.setMinimumPointsNumberPerVoxel(min_points_per_voxel);
        vfilter.filter(*cloud_filtered);
        emit filterResult(cloud_filtered, time.toc());
    }

} // namespace ct

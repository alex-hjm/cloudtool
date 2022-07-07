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

#include <pcl/filters/impl/local_maximum.hpp>
#include <pcl/filters/impl/project_inliers.hpp>

namespace ct
{

    void Filters::PassThrough(const std::string& field_name, float limit_min, float limit_max)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::PassThrough<PointXYZRGBN> pfilter;
        pfilter.setInputCloud(cloud_);
        pfilter.setFilterFieldName(field_name);
        pfilter.setFilterLimits(limit_min, limit_max);
        pfilter.setNegative(negative_);
        pfilter.filter(*cloud_filtered);
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::VoxelGrid(float lx, float ly, float lz)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::VoxelGrid<PointXYZRGBN> vfilter;
        vfilter.setInputCloud(cloud_);
        vfilter.setLeafSize(lx, ly, lz);
        vfilter.setFilterLimitsNegative(negative_);
        vfilter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::ApproximateVoxelGrid(float lx, float ly, float lz)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::ApproximateVoxelGrid<PointXYZRGBN> avfilter;
        avfilter.setInputCloud(cloud_);
        avfilter.setLeafSize(lx, ly, lz);
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

    void Filters::StatisticalOutlierRemoval(int nr_k, double stddev_mult)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::StatisticalOutlierRemoval<PointXYZRGBN> sfilter;
        sfilter.setInputCloud(cloud_);
        sfilter.setMeanK(nr_k);
        sfilter.setStddevMulThresh(stddev_mult);
        sfilter.setNegative(negative_);
        sfilter.filter(*cloud_filtered);
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::RadiusOutlierRemoval(double radius, int min_pts)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::RadiusOutlierRemoval<PointXYZRGBN> rfilter;
        rfilter.setInputCloud(cloud_);
        rfilter.setRadiusSearch(radius);
        rfilter.setMinNeighborsInRadius(min_pts);
        rfilter.setNegative(negative_);
        rfilter.filter(*cloud_filtered);
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::ConditionalRemoval(ConditionBase::Ptr con)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::ConditionalRemoval<PointXYZRGBN> bfilter;
        bfilter.setInputCloud(cloud_);
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

    void Filters::GridMinimum(const float resolution)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::GridMinimum<PointXYZRGBN> filter(resolution);
        filter.setInputCloud(cloud_);
        filter.setResolution(resolution);
        filter.setNegative(negative_);
        filter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::LocalMaximum(float radius)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::LocalMaximum<PointXYZRGBN> filter;
        filter.setInputCloud(cloud_);
        filter.setRadius(radius);
        filter.setNegative(negative_);
        filter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::ShadowPoints(float threshold)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::ShadowPoints<PointXYZRGBN, PointXYZRGBN> rfilter;
        rfilter.setInputCloud(cloud_);
        rfilter.setNormals(cloud_);
        rfilter.setThreshold(threshold);
        rfilter.setNegative(negative_);
        rfilter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::DownSampling(float radius)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::VoxelGrid<PointXYZRGBN> vfilter;
        vfilter.setInputCloud(cloud_);
        vfilter.setLeafSize(radius, radius, radius);
        vfilter.setFilterLimitsNegative(negative_);
        vfilter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::UniformSampling(float radius)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

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

    void Filters::RandomSampling(int sample, int seed)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::RandomSample<PointXYZRGBN> rfilter;
        rfilter.setInputCloud(cloud_);
        rfilter.setSample(sample);
        rfilter.setSeed(seed);
        rfilter.setNegative(negative_);
        rfilter.filter(*cloud_filtered);
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::ReSampling(float radius, int polynomial_order)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::MovingLeastSquares<PointXYZRGBN, PointXYZRGBN> mfilter;
        mfilter.setInputCloud(cloud_);
        mfilter.setSearchRadius(radius);
        if (!cloud_->hasNormals())mfilter.setComputeNormals(true);
        mfilter.setPolynomialOrder(polynomial_order);
        mfilter.setSearchMethod(tree);
        mfilter.setNumberOfThreads(14);
        mfilter.process(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }
    
    void Filters::SamplingSurfaceNormal(int sample, int seed, float ratio)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

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

    void Filters::NormalSpaceSampling(int sample, int seed, int bin)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::NormalSpaceSampling<PointXYZRGBN, PointXYZRGBN> filter;
        filter.setInputCloud(cloud_);
        filter.setNormals(cloud_);
        filter.setSample(sample);
        filter.setSeed(seed);
        filter.setBins(bin, bin, bin);
        filter.setNegative(negative_);
        filter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::Convolution3D(float sigma, float sigma_coefficient, float threshold, double radius)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());
        pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

        pcl::filters::GaussianKernel<PointXYZRGBN, PointXYZRGBN> kernel;
        kernel.setSigma(sigma);
        kernel.setThresholdRelativeToSigma(sigma_coefficient);
        kernel.setThreshold(threshold);

        pcl::filters::Convolution3D< PointXYZRGBN, PointXYZRGBN, pcl::filters::GaussianKernel<PointXYZRGBN, PointXYZRGBN>> convolution;
        convolution.setInputCloud(cloud_);
        convolution.setKernel(kernel);
        convolution.setNumberOfThreads(14);
        convolution.setSearchMethod(tree);
        convolution.setRadiusSearch(radius);
        convolution.convolve(*cloud_filtered);
        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::CropBox(const Eigen::Vector4f& min_pt, const Eigen::Vector4f& max_pt, const Eigen::Affine3f& transform)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::CropBox<PointXYZRGBN> cb;
        cb.setInputCloud(cloud_);
        cb.setMin(min_pt);
        cb.setMax(max_pt);
        cb.setTransform(transform);
        cb.setNegative(negative_);
        cb.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::CropHull(const std::vector<pcl::Vertices>& polygons, int dim, bool crop_outside)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::CropHull<PointXYZRGBN> ch;
        ch.setInputCloud(cloud_);
        ch.setHullIndices(polygons);
        ch.setDim(dim);
        ch.setCropOutside(crop_outside);
        ch.setNegative(negative_);
        ch.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::FrustumCulling(const Eigen::Matrix4f& camera_pose, float hfov, float vfov, float np_dist, float fp_dist)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::FrustumCulling<PointXYZRGBN> filter;
        filter.setInputCloud(cloud_);
        filter.setCameraPose(camera_pose);
        filter.setHorizontalFOV(hfov);
        filter.setVerticalFOV(vfov);
        filter.setNearPlaneDistance(np_dist);
        filter.setFarPlaneDistance(fp_dist);
        filter.setNegative(negative_);
        filter.filter(*cloud_filtered);

        emit filterResult(cloud_filtered, time.toc());
    }

    void Filters::MedianFilter(int window_size, float max_allowed_movement)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

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

    void Filters::PlaneClipper3D(const Eigen::Vector4f& plane_params)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

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

    void Filters::ProjectInliers(int type, const pcl::ModelCoefficients::Ptr& model, bool va)
    {
        TicToc time;
        time.tic();
        Cloud::Ptr cloud_filtered(new Cloud);
        cloud_filtered->setId(cloud_->id());

        pcl::ProjectInliers<PointXYZRGBN> filter;
        filter.setInputCloud(cloud_);
        filter.setModelType(type);
        filter.setModelCoefficients(model);
        filter.setCopyAllData(va);
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
} // namespace ct

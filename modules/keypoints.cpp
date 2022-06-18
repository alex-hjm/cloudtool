/**
 * @file keypoints.cpp
 * @author hjm (hjmalex@163.com)
 * @version 3.0
 * @date 2022-05-10
 */
#include "modules/keypoints.h"
#include "base/common.h"

#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/trajkovic_3d.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>

namespace ct
{

  void Keypoints::NarfKeypoint(const RangeImage* range_image, float support_size)
  {
    TicToc time;
    time.tic();
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage(range_image);
    narf_keypoint_detector.getParameters().support_size = support_size;

    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute(keypoint_indices);

    Cloud::Ptr keypoints(new Cloud);
    keypoints->points.resize(keypoint_indices.points.size());
    for (size_t i = 0; i < keypoint_indices.points.size(); ++i)
      keypoints->points[i].getVector3fMap() = range_image->points[keypoint_indices.points[i]].getVector3fMap();
    emit keypointsResult(keypoints, time.toc());
  }


  void Keypoints::HarrisKeypoint3D(int response_method, float radius, float threshold)
  {
    TicToc time;
    time.tic();
    Cloud::Ptr keypoints(new Cloud);
    pcl::PointCloud<PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<PointXYZI>);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

    pcl::HarrisKeypoint3D<PointXYZRGBN, PointXYZI, PointXYZRGBN> est;
    est.setInputCloud(cloud_);
    est.setSearchSurface(surface_);
    est.setNormals(cloud_);
    est.setSearchMethod(tree);
    est.setKSearch(k_);
    est.setRadiusSearch(radius_);
    est.setMethod(pcl::HarrisKeypoint3D<PointXYZRGBN, PointXYZI, PointXYZRGBN>::ResponseMethod(response_method + 1));
    est.setRadius(radius);
    est.setThreshold(threshold);
    est.setNumberOfThreads(12);
    est.compute(*keypoints_temp);

    pcl::copyPointCloud(*keypoints_temp, *keypoints);
    emit keypointsResult(keypoints, time.toc());
  }

  void Keypoints::ISSKeypoint3D(double resolutin, float angle)
  {
    TicToc time;
    time.tic();
    Cloud::Ptr keypoints(new Cloud);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

    pcl::ISSKeypoint3D<PointXYZRGBN, PointXYZRGBN, PointXYZRGBN> est;
    est.setInputCloud(cloud_);
    est.setSearchSurface(surface_);
    est.setNormals(cloud_);
    est.setSearchMethod(tree);
    est.setKSearch(k_);
    est.setRadiusSearch(radius_);
    est.setSalientRadius(6 * resolutin);
    est.setNonMaxRadius(4 * resolutin);
    est.setNormalRadius(4 * resolutin);
    est.setBorderRadius(4 * resolutin);
    est.setAngleThreshold(pcl::deg2rad(angle));
    est.setNumberOfThreads(12);
    est.compute(*keypoints);

    emit keypointsResult(keypoints, time.toc());
  }

  void Keypoints::SIFTKeypoint(float min_scale, int nr_octaves,
                               int nr_scales_per_octave, float min_contrast)
  {
    TicToc time;
    time.tic();
    Cloud::Ptr keypoints(new Cloud);
    pcl::PointCloud<PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<PointXYZRGB>);
    pcl::copyPointCloud(*cloud_, *cloud_xyzrgb);
    pcl::PointCloud<PointWithScale>::Ptr result(new pcl::PointCloud<PointWithScale>);
    pcl::search::KdTree<PointXYZRGB>::Ptr tree(new pcl::search::KdTree<PointXYZRGB>);

    pcl::SIFTKeypoint<PointXYZRGB, PointWithScale> est;
    est.setInputCloud(cloud_xyzrgb);
    //est.setSearchSurface(surface_);
    est.setSearchMethod(tree);
    est.setKSearch(k_);
    est.setRadiusSearch(radius_);
    est.setScales(min_scale, nr_octaves, nr_scales_per_octave);
    est.setMinimumContrast(min_contrast);
    est.compute(*result);

    pcl::copyPointCloud(*result, *keypoints);
    emit keypointsResult(keypoints, time.toc());
  }

  void Keypoints::TrajkovicKeypoint3D(int compute_method, int window_size,
                                      float first_threshold, float second_threshold)
  {
    TicToc time;
    time.tic();
    Cloud::Ptr keypoints(new Cloud);
    pcl::PointCloud<PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<PointXYZI>);
    pcl::search::KdTree<PointXYZRGBN>::Ptr tree(new pcl::search::KdTree<PointXYZRGBN>);

    pcl::TrajkovicKeypoint3D<PointXYZRGBN, PointXYZI, PointXYZRGBN> est;
    est.setInputCloud(cloud_);
    est.setSearchSurface(surface_);
    est.setNormals(cloud_);
    est.setSearchMethod(tree);
    est.setKSearch(k_);
    est.setRadiusSearch(radius_);
    est.setMethod(pcl::TrajkovicKeypoint3D<PointXYZRGBN, PointXYZI, PointXYZRGBN>::ComputationMethod(compute_method));
    est.setWindowSize(window_size);
    est.setFirstThreshold(first_threshold);
    est.setSecondThreshold(second_threshold);
    est.setNumberOfThreads(12);
    est.compute(*keypoints_temp);

    pcl::copyPointCloud(*keypoints_temp, *keypoints);
    emit keypointsResult(keypoints, time.toc());
  }

}  // namespace pca
